# Interface to Klipper micro-controller code
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from operator import mod
import sys, os, zlib, logging, math ,re
from klippy import Printer
import serialhdl, msgproto, pins, chelper, clocksync
import time
import csv
import binascii
from datetime import datetime

class error(Exception):
    pass

class MCU_trsync:
    REASON_ENDSTOP_HIT = 1
    REASON_COMMS_TIMEOUT = 2
    REASON_HOST_REQUEST = 3
    REASON_PAST_END_TIME = 4
    def __init__(self, mcu, trdispatch):
        self._mcu = mcu
        self._trdispatch = trdispatch
        self._reactor = mcu.get_printer().get_reactor()
        self._steppers = []
        self._trdispatch_mcu = None
        self._oid = mcu.create_oid()
        self._cmd_queue = mcu.alloc_command_queue()
        self._trsync_start_cmd = self._trsync_set_timeout_cmd = None
        self._trsync_trigger_cmd = self._trsync_query_cmd = None
        self._stepper_stop_cmd = None
        self._trigger_completion = None
        self._home_end_clock = None
        mcu.register_config_callback(self._build_config)
        printer = mcu.get_printer()
        printer.register_event_handler("klippy:shutdown", self._shutdown)
    def get_mcu(self):
        return self._mcu
    def get_oid(self):
        return self._oid
    def get_command_queue(self):
        return self._cmd_queue
    def add_stepper(self, stepper):
        if stepper in self._steppers:
            return
        self._steppers.append(stepper)
    def get_steppers(self):
        return list(self._steppers)
    def _build_config(self):
        mcu = self._mcu
        # Setup config
        mcu.add_config_cmd("config_trsync oid=%d" % (self._oid,))
        mcu.add_config_cmd(
            "trsync_start oid=%d report_clock=0 report_ticks=0 expire_reason=0"
            % (self._oid,), on_restart=True)
        # Lookup commands
        self._trsync_start_cmd = mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c", cq=self._cmd_queue)
        self._trsync_set_timeout_cmd = mcu.lookup_command(
            "trsync_set_timeout oid=%c clock=%u", cq=self._cmd_queue)
        self._trsync_trigger_cmd = mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c", cq=self._cmd_queue)
        self._trsync_query_cmd = mcu.lookup_query_command(
            "trsync_trigger oid=%c reason=%c",
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=self._oid, cq=self._cmd_queue)
        self._stepper_stop_cmd = mcu.lookup_command(
            "stepper_stop_on_trigger oid=%c trsync_oid=%c", cq=self._cmd_queue)
        # Create trdispatch_mcu object
        set_timeout_tag = mcu.lookup_command_tag(
            "trsync_set_timeout oid=%c clock=%u")
        trigger_tag = mcu.lookup_command_tag("trsync_trigger oid=%c reason=%c")
        state_tag = mcu.lookup_command_tag(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u")
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch_mcu = ffi_main.gc(ffi_lib.trdispatch_mcu_alloc(
            self._trdispatch, mcu._serial.serialqueue, # XXX
            self._cmd_queue, self._oid, set_timeout_tag, trigger_tag,
            state_tag), ffi_lib.free)
    def _shutdown(self):
        tc = self._trigger_completion
        if tc is not None:
            self._trigger_completion = None
            tc.complete(False)
    def _handle_trsync_state(self, params):
        if not params['can_trigger']:
            tc = self._trigger_completion
            if tc is not None:
                self._trigger_completion = None
                reason = params['trigger_reason']
                is_failure = (reason == self.REASON_COMMS_TIMEOUT)
                self._reactor.async_complete(tc, is_failure)
        elif self._home_end_clock is not None:
            clock = self._mcu.clock32_to_clock64(params['clock'])
            if clock >= self._home_end_clock:
                self._home_end_clock = None
                self._trsync_trigger_cmd.send([self._oid,
                                               self.REASON_PAST_END_TIME])
    def start(self, print_time, trigger_completion, expire_timeout):
        self._trigger_completion = trigger_completion
        self._home_end_clock = None
        clock = self._mcu.print_time_to_clock(print_time)
        expire_ticks = self._mcu.seconds_to_clock(expire_timeout)
        expire_clock = clock + expire_ticks
        report_ticks = self._mcu.seconds_to_clock(expire_timeout * .4)
        min_extend_ticks = self._mcu.seconds_to_clock(expire_timeout * .4 * .8)
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_mcu_setup(self._trdispatch_mcu, clock, expire_clock,
                                     expire_ticks, min_extend_ticks)
        self._mcu.register_response(self._handle_trsync_state,
                                    "trsync_state", self._oid)
        self._trsync_start_cmd.send([self._oid, clock, report_ticks,
                                     self.REASON_COMMS_TIMEOUT], reqclock=clock)
        for s in self._steppers:
            self._stepper_stop_cmd.send([s.get_oid(), self._oid])
        self._trsync_set_timeout_cmd.send([self._oid, expire_clock],
                                          reqclock=expire_clock)
    def set_home_end_time(self, home_end_time):
        self._home_end_clock = self._mcu.print_time_to_clock(home_end_time)
    def stop(self):
        self._mcu.register_response(None, "trsync_state", self._oid)
        self._trigger_completion = None
        if self._mcu.is_fileoutput():
            return self.REASON_ENDSTOP_HIT
        params = self._trsync_query_cmd.send([self._oid,
                                              self.REASON_HOST_REQUEST])
        for s in self._steppers:
            s.note_homing_end()
        return params['trigger_reason']

TRSYNC_TIMEOUT = 0.025
TRSYNC_SINGLE_MCU_TIMEOUT = 0.250

class MCU_endstop:
    RETRY_QUERY = 1.000
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._pin = pin_params['pin']
        self._pullup = pin_params['pullup']
        self._invert = pin_params['invert']
        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = None
        self._mcu.register_config_callback(self._build_config)
        self._trigger_completion = None
        self._rest_ticks = 0
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]
    def get_mcu(self):
        return self._mcu
    def add_stepper(self, stepper):
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith('stepper_'):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self._mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")
    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]
    def _build_config(self):
        # Setup config
        self._mcu.add_config_cmd("config_endstop oid=%d pin=%s pull_up=%d"
                                 % (self._oid, self._pin, self._pullup))
        self._mcu.add_config_cmd(
            "endstop_home oid=%d clock=0 sample_ticks=0 sample_count=0"
            " rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0"
            % (self._oid,), on_restart=True)
        # Lookup commands
        cmd_queue = self._trsyncs[0].get_command_queue()
        self._home_cmd = self._mcu.lookup_command(
            "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c"
            " rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c",
            cq=cmd_queue)
        self._query_cmd = self._mcu.lookup_query_command(
            "endstop_query_state oid=%c",
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            oid=self._oid, cq=cmd_queue)
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        clock = self._mcu.print_time_to_clock(print_time)
        rest_ticks = self._mcu.print_time_to_clock(print_time+rest_time) - clock
        self._rest_ticks = rest_ticks
        reactor = self._mcu.get_printer().get_reactor()
        self._trigger_completion = reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for trsync in self._trsyncs:
            trsync.start(print_time, self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        self._home_cmd.send(
            [self._oid, clock, self._mcu.seconds_to_clock(sample_time),
             sample_count, rest_ticks, triggered ^ self._invert,
             etrsync.get_oid(), etrsync.REASON_ENDSTOP_HIT], reqclock=clock)
        return self._trigger_completion
    def home_wait(self, home_end_time):
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0])
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.
        if self._mcu.is_fileoutput():
            return home_end_time
        params = self._query_cmd.send([self._oid])
        next_clock = self._mcu.clock32_to_clock64(params['next_clock'])
        return self._mcu.clock_to_print_time(next_clock - self._rest_ticks)
    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        return params['pin_value'] ^ self._invert

class MCU_digital_out:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._oid = None
        self._mcu.register_config_callback(self._build_config)
        self._pin = pin_params['pin']
        self._invert = pin_params['invert']
        self._start_value = self._shutdown_value = self._invert
        self._is_static = False
        self._max_duration = 2.
        self._last_clock = 0
        self._set_cmd = None
    def get_mcu(self):
        return self._mcu
    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration
    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        if is_static and start_value != shutdown_value:
            raise pins.error("Static pin can not have shutdown value")
        self._start_value = (not not start_value) ^ self._invert
        self._shutdown_value = (not not shutdown_value) ^ self._invert
        self._is_static = is_static
    def _build_config(self):
        if self._is_static:
            self._mcu.add_config_cmd("set_digital_out pin=%s value=%d"
                                     % (self._pin, self._start_value))
            return
        self._mcu.request_move_queue_slot()
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd(
            "config_digital_out oid=%d pin=%s value=%d default_value=%d"
            " max_duration=%d"
            % (self._oid, self._pin, self._start_value, self._shutdown_value,
               self._mcu.seconds_to_clock(self._max_duration)))
        self._mcu.add_config_cmd("update_digital_out oid=%d value=%d"
                                 % (self._oid, self._start_value),
                                 on_restart=True)
        cmd_queue = self._mcu.alloc_command_queue()
        self._set_cmd = self._mcu.lookup_command(
            "queue_digital_out oid=%c clock=%u on_ticks=%u", cq=cmd_queue)
    def set_digital(self, print_time, value):
        clock = self._mcu.print_time_to_clock(print_time)
        # self._last_clock = self._mcu.print_time_to_clock(print_time - 0.2)
        self._set_cmd.send([self._oid, clock, (not not value) ^ self._invert],
                           minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock

class MCU_pwm:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._hardware_pwm = False
        self._cycle_time = 0.100
        self._max_duration = 5
        self._oid = None
        self._mcu.register_config_callback(self._build_config)
        self._pin = pin_params['pin']
        self._invert = pin_params['invert']
        self._start_value = self._shutdown_value = float(self._invert)
        self._is_static = False
        self._last_clock = self._last_cycle_ticks = 0
        self._pwm_max = 0.
        self._set_cmd = self._set_cycle_ticks = None
        
    def get_mcu(self):
        return self._mcu
    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self._cycle_time = cycle_time
        self._hardware_pwm = hardware_pwm
    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        if is_static and start_value != shutdown_value:
            raise pins.error("Static pin can not have shutdown value")
        if self._invert:
            start_value = 1. - start_value
            shutdown_value = 1. - shutdown_value
        self._start_value = max(0., min(1., start_value))
        self._shutdown_value = max(0., min(1., shutdown_value))
        self._is_static = is_static
    def _build_config(self):
        cmd_queue = self._mcu.alloc_command_queue()
        curtime = self._mcu.get_printer().get_reactor().monotonic()
        printtime = self._mcu.estimated_print_time(curtime)
        self._last_clock = self._mcu.print_time_to_clock(printtime + 0.200)
        cycle_ticks = self._mcu.seconds_to_clock(self._cycle_time)
        if self._hardware_pwm:
            self._pwm_max = self._mcu.get_constant_float("PWM_MAX")
            if self._is_static:
                self._mcu.add_config_cmd(
                    "set_pwm_out pin=%s cycle_ticks=%d value=%d"
                    % (self._pin, cycle_ticks,
                       self._start_value * self._pwm_max))
                return
            self._mcu.request_move_queue_slot()
            self._oid = self._mcu.create_oid()
            self._mcu.add_config_cmd(
                "config_pwm_out oid=%d pin=%s cycle_ticks=%d value=%d"
                " default_value=%d max_duration=%d"
                % (self._oid, self._pin, cycle_ticks,
                   self._start_value * self._pwm_max,
                   self._shutdown_value * self._pwm_max,
                   self._mcu.seconds_to_clock(self._max_duration)))
            svalue = int(self._start_value * self._pwm_max + 0.5)
            self._mcu.add_config_cmd("queue_pwm_out oid=%d clock=%d value=%d"
                                     % (self._oid, self._last_clock, svalue),
                                     on_restart=True)
            self._set_cmd = self._mcu.lookup_command(
                "queue_pwm_out oid=%c clock=%u value=%hu", cq=cmd_queue)
            return
        # Software PWM
        if self._shutdown_value not in [0., 1.]:
            raise pins.error("shutdown value must be 0.0 or 1.0 on soft pwm")
        if self._is_static:
            self._mcu.add_config_cmd("set_digital_out pin=%s value=%d"
                                     % (self._pin, self._start_value >= 0.5))
            return
        self._mcu.request_move_queue_slot()
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd(
            "config_digital_out oid=%d pin=%s value=%d"
            " default_value=%d max_duration=%d"
            % (self._oid, self._pin, self._start_value >= 1.0,
               self._shutdown_value >= 0.5,
               self._mcu.seconds_to_clock(self._max_duration)))
        self._mcu.add_config_cmd(
            "set_digital_out_pwm_cycle oid=%d cycle_ticks=%d"
            % (self._oid, cycle_ticks))
        self._last_cycle_ticks = cycle_ticks
        svalue = int(self._start_value * cycle_ticks + 0.5)
        self._mcu.add_config_cmd(
            "queue_digital_out oid=%d clock=%d on_ticks=%d"
            % (self._oid, self._last_clock, svalue), is_init=True)
        self._set_cmd = self._mcu.lookup_command(
            "queue_digital_out oid=%c clock=%u on_ticks=%u", cq=cmd_queue)
        self._set_cycle_ticks = self._mcu.lookup_command(
            "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u", cq=cmd_queue)
    def set_pwm(self, print_time, value, cycle_time=None):
        clock = self._mcu.print_time_to_clock(print_time)
        # minclock = self._mcu.print_time_to_clock(print_time - 0.2)
        minclock = self._last_clock
        self._last_clock = clock 
        if self._invert:
            value = 1. - value
        if self._hardware_pwm:
            # print("try set pwm value via hardware pwm")
            v = int(max(0., min(1., value)) * self._pwm_max + 0.5)
            self._set_cmd.send([self._oid, clock, v],
                               minclock=minclock, reqclock=clock)
            return
        # Soft pwm update
        # print("try set pwm value via software pwm %.3f"%value)
        if cycle_time is None:
            cycle_time = self._cycle_time
        cycle_ticks = self._mcu.seconds_to_clock(cycle_time)
        
        if cycle_ticks != self._last_cycle_ticks:
            self._set_cycle_ticks.send([self._oid, cycle_ticks],
                                       minclock=minclock, reqclock=clock)
            self._last_cycle_ticks = cycle_ticks
        on_ticks = int(max(0., min(1., value)) * float(cycle_ticks) + 0.5)
        # self._mcu.ghead.nprintf("cycle_time = %0.1f,cycle_ticks = %d on_ticks = %d"%(cycle_time,cycle_ticks,on_ticks))
        self._set_cmd.send([self._oid, clock, on_ticks],
                           minclock=minclock, reqclock=clock)
COMM_DELAY_OFFSET = 0.001
MAX_TOOL_NUM = 2
GHEAD_ISP_MODE = 0
GHEAD_APP_MODE = 1
UPDATE_SIZE_PERPACKAGE = 256
RECORD_GHEAD_MSG = 1
GHEAD_FIRMWARE_DIR = "/opt/Raise3D/temp/Pro3-HCB.mcufirm"

class wait_event:
    def __init__(self,reactor,handle):

        pass
class MCU_ghead:
    def __init__(self,mcu,ptr,printer,reactor = None):
        self.reactor = reactor
        self._mcu = mcu
        self._ptr = ptr
        self._oid = self._callback = None
        self._tempv = {}
        self._ksensor_s  = {}
        self._location_s = {}
        self._ghead_in_s = {}
        self.fan_speed = {}
        self.set_fanspeed = None
        self.start_ghead_moitor = None
        self.set_heater_s = None
        self.handletempcallback = {}
        self.printer = printer
        self.gcode = None
        self.endstop = None
        self.printer_type = "Pro3"
        self.m5802_list = []
        self.raw_dat_list = []
        self.raw_dat_ptr = 0
        self.psd_num = 0.
        self.firmware_dir = GHEAD_FIRMWARE_DIR
        self.wait_event_timer = self.reactor.register_timer(
                self.wait_event_timeout_handle, self.reactor.NEVER)
        self.left_location_shake_timer = self.reactor.register_timer(
                self.left_location_handle, self.reactor.NEVER)
        self.right_location_shake_timer = self.reactor.register_timer(
                self.right_location_handle, self.reactor.NEVER)
        self.wait_event = "idle"
        self.wait_val = 0
        self.wait_ptr = 0
        self.wait_mode = 0
        self.wait_response_count = 0
        self.check_version = 0
        self.first_power_on = False

        self.ghead_firmware_list = []
        self.firmware_buff = []
        self.package_n = 0
        self.package_ptr = 0
        self.errorcount = [0,0]
        self.errortemp  = [0.,0.]
        self.x_retry_count = 0
        self.y_retry_count = 0
        self.retry_count = 0

        self.i2c_test_count = 0
        self.is_i2c_test    = 0

    def wait_event_timeout_handle(self,eventtime):
        if self.wait_event == "idle":
            self.wait_response_count = 0
            pass
        elif self.wait_event == "caculate_start_response":
            self.wait_response_count = self.wait_response_count + 1
            if self.wait_response_count <= 3:
                self.set_normal_ghead_ctrl(ord('P')+ord('R'),0,self.wait_val)
                return eventtime + 1
            self.nprintf("no response when try to start caculate")
            self.wait_response_count = 0
            pass
        elif self.wait_event == "messure_start_response":
            self.wait_response_count = self.wait_response_count + 1
            if self.wait_response_count <= 3:
                self.set_normal_ghead_ctrl(self.wait_mode,self.wait_ptr,self.wait_val)
                return eventtime + 1
            self.nprintf("no response when try to start messure")
            self.wait_response_count = 0
        elif self.wait_event == "messure_stop_response":
            self.wait_response_count = self.wait_response_count + 1
            if self.wait_response_count <= 3:
                self.set_normal_ghead_ctrl(self.wait_mode,self.wait_ptr,self.wait_val)
                return eventtime + 1
            self.nprintf("no response when try to stop messure")
            self.wait_response_count = 0
        return self.reactor.NEVER
    def cancel_wait_event(self):
        self.wait_event = "idle"
    def set_printer_type(self,type):
        self.printer_type = type
    def register_temphandle(self,ptr,fuc):
        if ptr in self.handletempcallback:
            return
        self.handletempcallback[ptr] = fuc
    def setup_endstop(self,endstop):
        self.endstop = endstop
    def cmd_M280(self,gcmd):
        servo_val = gcmd.get_int('S',None)
        if servo_val != None:
            if servo_val == 10:
                self.probe_lower(self.endstop)
            elif servo_val == 200:
                self.probe_raise(self.endstop)
        pass
    def cmd_M5106(self,gcmd):
        cmdline = str(gcmd._commandline)
        # ptr = gcmd.get_int('H',0)
        # mode = ord('A')
        # addr = gcmd.get_int('A',0)
        # size = gcmd.get_int('S',0)
        # val = addr << 8 | size
        # self.set_normal_ghead_ctrl(mode,ptr,val) 
        # self.nprintf("val = %d"%val)
        m5106_buff = list(cmdline)
        self.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,m5106_buff))
        pass
    # def cmd_M5110(self,gcmd):
    #     cmdlist = gcmd._params
    #     if 'S' in cmdlist:
    #         val = int(cmdlist['S'])
    #         if val > 0:
    #             self.nprintf("try to start i2c read test")
    #             if 'T' in cmdlist and 'P' in cmdlist:
    #                 period =  int(cmdlist['P'])
    #                 count  =  int(cmdlist['T'])
    #             else:
    #                 period = 1
    #                 count  = 1800
    #             self.is_i2c_test = True
    #         else:
    #             self.nprintf("try to stop i2c read test")
    #             self.is_i2c_test = False
    #         pass
    #     pass
    def cmd_M5108(self,gcmd):
        cmdline = str(gcmd._commandline)
        _buff = list(cmdline)
        self.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,_buff))
    def cmd_M5107(self,gcmd):
        cmdline = str(gcmd._commandline)
        cmdl = cmdline.split(" ")
        if len(cmdl) > 3:
            if cmdl[3][0] == 'D':
                mode = ord('D')
            else:
                mode = ord('H')
            gh_str = str(cmdl[3][1:])
            buff = list(gh_str)
            if cmdl[1][0] == 'H':
                g_ptr = int(cmdl[1][1:])
            else:
                g_ptr = 0
            if cmdl[2][0] == 'A':
                addr  = int(cmdl[2][1:])
            else:
                addr = 0
            self.nprintf("gptr = %d addr %d str = %s"%(g_ptr,addr,str(gh_str)))
        #     self.set_len_dat_cmd.send((g_ptr,mode,addr,buff))
            m5107_buff = list(cmdline)
            self.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,m5107_buff))
        pass
    def query_temp(self,ptr):
        self.set_normal_ghead_ctrl(ord('Q') + ord('T'),ptr,0)
    def start_heat(self,ptr):
        self.set_normal_ghead_ctrl(ord('H'),ptr,1) 
    def stop_heat(self,ptr):
        self.set_normal_ghead_ctrl(ord('H'),ptr,0) 
    def ctrl_ghead(self,gcmd):
        cmd = gcmd.get_command()
        cmdlist = gcmd._params
        if cmd == "M5100":
            if "G" in cmdlist :
                ptr = int(cmdlist["G"])
                if "H" in cmdlist:
                    # switch heater s
                    val = int(cmdlist["H"])
                    # self.set_heater_s_cmd(ptr,val)
                    self.set_normal_ghead_ctrl(ord('H'),ptr,val) 
                elif "F" in cmdlist:
                    # set fan speed 
                    val = int(cmdlist["F"])
                    
                    if ptr < 2:
                        self.set_fanspeed_cmd(ptr,val)
                    else:
                        self.set_normal_ghead_ctrl(ord('F'),ptr,val)
                elif "S" in cmdlist: 
                    # set servo
                    val = int(cmdlist["S"])
                    self.nprintf("ghead ctrl T%d" %(val))
                    self.set_normal_ghead_ctrl(ord('S'),0,val)
                elif "T" in cmdlist:
                    # set magnet 0 low 1 up
                    val = int(cmdlist["T"])
                    # self.set_normal_ghead_ctrl
                    # (ord('T'),0,val)
                    if val == 0:
                        self.probe_lower(self.endstop)
                    else:
                        self.probe_raise(self.endstop)
                elif "B" in cmdlist:
                    val = int(cmdlist["B"])
                    if val == 9600:
                        self.set_normal_ghead_ctrl(ord('B'),0,0)
                    elif val == 19200:
                        self.set_normal_ghead_ctrl(ord('B'),0,1)
                    elif val == 38400:
                        self.set_normal_ghead_ctrl(ord('B'),0,2)
                    elif val == 57600:
                        self.set_normal_ghead_ctrl(ord('B'),0,3)
                    elif val == 115200:
                        self.set_normal_ghead_ctrl(ord('B'),0,4)
                    else:
                        self.set_normal_ghead_ctrl(ord('B'),0,4)   
                    print ("try to set baud %d"%val)
                elif "P" in cmdlist:
                    val = int(cmdlist["P"])
                    self.set_normal_ghead_ctrl(ord('P'),ptr,val) 
                elif "U" in cmdlist:
                    val = int(cmdlist["U"])
                    self.set_normal_ghead_ctrl(ord('U'),ptr,val) 
                elif "A" in cmdlist:
                    val = int(cmdlist["A"])
                    self.set_normal_ghead_ctrl(ord('A'),ptr,val) 
                elif "C" in cmdlist:
                    val = int(cmdlist["C"])
                    self.set_normal_ghead_ctrl(ord('C'),ptr,val)
                elif "Z" in cmdlist:
                    val = int(cmdlist["Z"])
                    self.set_normal_ghead_ctrl(ord('Z'),ptr,val) 
                elif "U" in cmdlist:
                    val = int(cmdlist["U"])
                    self.set_normal_ghead_ctrl(ord('U'),ptr,val) 
                elif "V" in cmdlist:
                    self.set_normal_ghead_ctrl(ord('V'),ptr,0) 
                elif "CV" in cmdlist:
                    self.set_normal_ghead_ctrl((ord('V') + ord('C')),ptr,0) 
                elif "RH" in cmdlist:
                    val = int(cmdlist["RH"])
                    if val < 2:
                        self.set_normal_ghead_ctrl((ord('R') + ord('H')),ptr,val) 
                    else:
                        gcmd.respond_raw("ivalid parameters!!")
                elif "K" in cmdlist:
                    mode = ord('B') + ord('R') + ord('T')
                    self.set_normal_ghead_ctrl(mode,ptr,val) 
                    self.nprintf("try to reboot moveboard")
                elif "SCAN" in cmdlist:
                    self.nprintf("try to query_ghead_in")
                    val = (1,)
                    self.query_ghead_in_cmd.send(val)
                elif "CT" in cmdlist:
                    if ptr < 2 :
                        count = int(cmdlist['CT'])
                        self.errorcount[ptr] = count
                        
                        if count > 0 and 'E' in cmdlist:
                            self.errortemp[ptr]  = float(cmdlist['E'])
                            self.nprintf("try to simulate temp on heater %d count %d fake temp %.1f "%(ptr,count,self.errortemp[ptr]))
                        else:
                            self.nprintf("reset simulate temp on %d "%(ptr,))
                            self.errortemp[ptr]  = 0
                else:
                    pass
            elif "CV" in cmdlist:
                self.set_normal_ghead_ctrl((ord('V') + ord('C')),0,0) 
            elif "V" in cmdlist:
                self.set_normal_ghead_ctrl(ord('V'),0,0) 
            elif "CB" in cmdlist:
                cmdline = str(gcmd._commandline)
                _buff = list(cmdline)
                self.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,_buff))
            elif "RESET" in cmdlist:
                self.nprintf("try remote reset")
                self._mcu._serial.try_remote_reset()
                pass
            elif "R" in cmdlist:
                val = int(cmdlist["R"])
                self.raw_dat_list = []
                self.raw_dat_ptr = 0
                self.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,"M5801 R%d"%val))
                pass
            elif "BR" in cmdlist:
                self.nprintf("psd data is bad! try calibrate again!")
                self.printer.send_event("ghead:response","messure_reshake_y")
    def transmit_cmd_to_ghead(self,cmd):
        _buff = list(cmd)
        self.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,_buff))
    def probe_raise(self,endstop = None):
        self.set_normal_ghead_ctrl(ord('T'),0,1)
        if endstop != None:
            pass         
    def probe_lower(self,endstop = None,wait = False):
        self.set_normal_ghead_ctrl(ord('T'),0,0)
        count = 5
        if wait == True and endstop == None:
            endstop = self.endstop
        if endstop != None :
            while True:
                self.reactor.pause(self.reactor.monotonic()+0.5)
                toolhead = self.printer.lookup_object('toolhead')
                print_time = toolhead.get_last_move_time()
                res = endstop.query_endstop(print_time)
                if res == 1:
                    count -=1
                    if count > 0:
                        self.set_normal_ghead_ctrl(ord('T'),0,0)
                    else:
                        return 
                else:
                    return 
    def switch_tool(self,toolnum):
        if toolnum < MAX_TOOL_NUM:
            self.set_normal_ghead_ctrl(ord('S'),0,toolnum) 
        else:
            pass
    def get_mcu(self):
        return self._mcu
    def get_tempv(self,ptr):
        return self.tempv[ptr]
    def get_ksensor_s(self):
        return self._ksensor_s
    def get_ghead_in_s(self):
        return self._ghead_in_s
    def nprintf(self,msg):
        gcode = self.printer.lookup_object('gcode')
        res = gcode.respond_raw(msg)
    def movement_handle(self,gcmd):
        self.nprintf("test reboot..")
        self.nprintf(str(gcmd._params))
        cmd = gcmd.get('REBOOT',None)
        if cmd != None:
            if cmd == 'update':
                mode = ord('B') + ord('R') + ord('T')
                self.set_normal_ghead_ctrl(mode,0,0)
                self.nprintf("try to reboot to update mode ??")
            elif cmd == "reset":
                self.nprintf("just reset test,.....")
    def cmd_M3019(self,gcmd):
        cmdlist = gcmd._params
        firware_name = gcmd.get('P',None)
        if firware_name != None:
            if '/' in firware_name:
                if os.path.exists(firware_name):
                    self.firmware_dir = firware_name
                    self.nprintf("new firmarename = %s"%(self.firmware_dir))
        else:
            self.nprintf("default firmarename = %s "%(self.firmware_dir))
        if 'H' in cmdlist:
            cmd_val = int(cmdlist['H'])
            if cmd_val == 0:
                # reset to app mode
                self.set_normal_ghead_ctrl((ord('U') + ord('G')),2,0) 
            elif cmd_val == 1:
                if self.load_ghead_firmware(self.firmware_dir) == True:
                    self.set_normal_ghead_ctrl((ord('U') + ord('G')),1,0)
                else:
                    self.gcode._respond_error("Fail to load the right firmware file,please check the direction")
            elif cmd_val == 2:
                self.set_normal_ghead_ctrl((ord('U') + ord('G')),3,0) 
            elif cmd_val == 3:
                self.set_normal_ghead_ctrl((ord('U') + ord('G')),3,1) 
            elif cmd_val == 4:
                self.load_ghead_firmware(self.firmware_dir)
    def cmd_M5101(self,gcmd):
        cmdlist = gcmd._params
        if 'P' in cmdlist:
            filename = gcmd.get('P').decode("utf-8")
            # try to load file
            self.load_ghead_firmware(filename)
        if 'D' in cmdlist:
            ptr = int(cmdlist['D'])
            self.debug_firmware_list(ptr) 
        if 'S' in cmdlist:
            val = int(cmdlist['S'])
            if val == 1:
                # start update
                pass
            else:
                pass
            pass
        if 'T' in cmdlist:
            val = int(cmdlist['T'])
            self.send_firmware_package(val)
        pass
    def send_firmware_package(self,ptr):
        start_pos = ptr*UPDATE_SIZE_PERPACKAGE
        buff = self.firmware_buff[start_pos:start_pos+48]
        self.set_len_dat_cmd.send((0,(ord('U')),0,list(buff)))

    def load_ghead_firmware(self,filename):
        # open bin file
        if not os.path.exists(filename) :
            self.nprintf("can't find file")
            self.nprintf("M3020 S1")
            return False
        self.ghead_firmware_list = []
        unit_len = UPDATE_SIZE_PERPACKAGE
        bsize = os.path.getsize(filename)
        if bsize == 0:
            self.nprintf("empty file!")
            self.nprintf("M3020 S2")
            return False
        self.nprintf("we have %d unit_len"%unit_len)
        remains = bsize % unit_len
        if remains == 0:
            package_n = bsize/unit_len
        else:
            package_n = bsize/unit_len + 1
        if package_n >= 255 :
            self.nprintf("file too large!")
            self.nprintf("M3020 S2")
            return False
        self.set_normal_ghead_ctrl(ord('U') + ord('G') ,0,package_n) 
        position_b = 0
        self.nprintf("M3020 T%d"%package_n)
        self.nprintf("filename = %s size= %d package n = %d"%(filename,bsize,package_n))
        self.nprintf("we have %d remains"%remains)
        self.firmware_buff = []
        with open(filename, "r") as binfile:
            for i in range(package_n*UPDATE_SIZE_PERPACKAGE):
                if i < bsize:
                    data = binfile.read(1)
                    self.firmware_buff.append(data)
                else:
                    self.firmware_buff.append(0)
                # binfile.seek(position_b)
                # if i < package_n - 1:
                #     data = list(binfile.read(unit_len))
                #     self.ghead_firmware_list.append(data)
                #     position_b = position_b + unit_len
                # elif i == (package_n - 1) and remains == 0:
                #     data = list(binfile.read(unit_len))
                #     self.ghead_firmware_list.append(data)
                #     position_b = position_b + unit_len
                # elif i == (package_n - 1) and remains > 0: 
                #     pdata = list(binfile.read(remains))
                #     for j in range(remains,unit_len):
                #         pdata.append(0)
                #     self.ghead_firmware_list.append(pdata)
                #     position_b = position_b + unit_len
        binfile.close()
        self.package_n = package_n
        return True

    def debug_firmware_list(self,ptr):
        if self.package_n == 0:
            self.nprintf("firmware has not been loaded")
            return
        if ptr < self.package_n:
            str_b = "package ptr = %d"%ptr
            start_pos = ptr*UPDATE_SIZE_PERPACKAGE
            buff = self.firmware_buff[start_pos:start_pos+32]
            strt = ""
            for c in buff:
                strt = strt + str(binascii.b2a_hex(c))[2:-1] 
            self.nprintf(strt)
        else:
            self.nprintf("exceed range %d:%d"%(ptr,self.package_n))
        pass
    def ghead_start(self,ptr):
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('M5100',self.ctrl_ghead,desc = None)
        self.gcode.register_command('M5106',self.cmd_M5106,desc = None)
        self.gcode.register_command('M5107',self.cmd_M5107,desc = None)
        self.gcode.register_command('M5108',self.cmd_M5108,desc = None)
        self.gcode.register_command('M5101',self.cmd_M5101,desc = None)
        self.gcode.register_command('M3019',self.cmd_M3019,desc = None)
        # self.gcode.register_command('M5110',self.cmd_M5110,desc = None)
        self.gcode.register_command('M280',self.cmd_M280,desc = None)
        self.gcode.register_command('MBOARD',self.movement_handle,desc = None)
        logging.info("ghead service starting..")
        if self._ptr == 0 :
            self._tempv['left']=0.0
            self._ksensor_s['left']=False
            self._location_s['left']=False
            self._ghead_in_s['left']=False
            self.fan_speed['left']=0
        elif self._ptr == 1 :
            self._tempv['right']=0.0
            self._ksensor_s['right']=False
            self._location_s['right']=False
            self._ghead_in_s['right']=False
            self.fan_speed['right']=0
        elif self._ptr == 2:
            self._tempv['left']=0.0
            self._tempv['right']=0.0
            self._ksensor_s['left']=False
            self._ksensor_s['right']=False
            self._location_s['left']=False
            self._location_s['right']=False
            self._ghead_in_s['left']=False
            self._ghead_in_s['right']=False
            self.fan_speed['left']=0
            self.fan_speed['right']=0
    
        self._mcu.register_response(self.handle_ghead_temp,
                                    "ghead_temp")
        self._mcu.register_response(self.handle_ghead_sensor_s,
                                    "ghead_ksensor_s")
        self._mcu.register_response(self.handle_ghead_location_s,
                                    "ghead_location_s")
        self._mcu.register_response(self.handle_ghead_in_s,
                                    "ghead_in_s")
        self._mcu.register_response(self.handle_setfan_res,
                                    "ghead_fanset_s")
        self._mcu.register_response(self.handle_respond,
                                    "ghead_respond_s")
        self._mcu.register_response(self.handle_ghead_slen,
                                    "ghead_slen")
        cmd_queue = self._mcu.alloc_command_queue()
        self.set_fanspeed = self._mcu.lookup_command(
            "ghead_set_fan_speed ptr=%c fans=%u",cq=cmd_queue)
        self.start_ghead_moitor = self._mcu.lookup_command(
            "ghead_start_monitor_ghead ptr=%c",cq=cmd_queue)
        self.set_heater_s = self._mcu.lookup_command(
            "ghead_set_heater_s ptr=%c h_s=%u",cq=cmd_queue)
        self.set_normal_cmd = self._mcu.lookup_command(
            "ghead_set_normal_cmd mode=%c ptr=%c h_s=%u",cq=cmd_queue)
        self.set_len_dat_cmd = self._mcu.lookup_command(
            "ghead_set_len_dat ptr=%c mode=%c addr=%c buff=%*s",cq=cmd_queue)
        self.query_ghead_in_cmd = self._mcu.lookup_command(
            "ghead_query_in ptr=%c",cq=cmd_queue)
    def set_fanspeed_cmd(self,ptr = 0,speed = 100):
        val = (ptr,speed)
        self.set_fanspeed.send(val)
    def set_heater_s_cmd(self,ptr = 0,sta = 1):
        val = (ptr,sta)
        self.set_heater_s.send(val)
    def set_normal_ghead_ctrl(self,mode = ord('H'),ptr = 0, hs = 0):
        val = (mode,ptr,hs)
        self.set_normal_cmd.send(val)
    def start_monitor_ghead(self,ptr = 0):
        val = (ptr,)
        self.start_ghead_moitor.send(val)
    def response_wait(self,event,mode,val,ptr):
        self.wait_event = event
        self.wait_val = val
        self.wait_ptr = ptr
        self.wait_mode = mode
        eventtime = self.reactor.monotonic()
        self.wait_response_count = 0
        self.reactor.update_timer(self.wait_event_timer,eventtime + 1)
        pass
    def record_msg_from_ghead(self,msg):
        if RECORD_GHEAD_MSG == 1:
            with open("ghead_record", "a") as record_file:
                date = datetime.now().strftime('%Y-%m-%d--%H:%M:%S')
                log = "[%s]%s\n"%(str(date),msg)
                record_file.write(log)
    def handle_ghead_slen(self,params):
        mode = int(params['mode'])
        ptr  = int(params['gh_ptr'])
        addr = int(params['addr'])
        if mode == (ord('G') + ord('H')):
            self.nprintf(str(params['buff']))
        elif mode == ord('V'):
            self.nprintf("MCB V%s"%(str(params['buff'],)))
        elif mode == (ord('V') + ord('C')):
            self.nprintf("HCB V%s"%(str(params['buff'],)))
        elif mode == ord('U'):
            msg = str(params['buff']).strip()
            self.nprintf(msg)
        elif mode == ord('P'):
            msg = str(params['buff']).strip()
            # self.nprintf(msg)
            cmd_msg = msg.split(" ")
            if cmd_msg[0] == "M5802":
                psd_params = { part[0]: part[1:].strip()
                                    for part in cmd_msg }
                if 'H' in psd_params:
                    head_ptr = int(psd_params['H'])
                else:
                    head_ptr = 1
                if head_ptr == 0:
                    headname = "left"
                else:
                    headname = "right"
                if 'P' in psd_params:
                    psd_msg = str(psd_params["P"])
                    psdv = psd_msg.split(':')
                    if int(psdv[0]) == 0:
                        self.m5802_list = []
                        self.m5802_list.append(psdv)
                    else:
                        self.m5802_list.append(psdv)
                    transmit_pro = float(psdv[0])*100/self.psd_num
                    self.nprintf("%s board transmit proc: %.1f"%(headname,transmit_pro,))
                    # if "end":
                    #     file_name = "file_{}.csv".format(int(time.time()))
                    #     with open(file_name, "w") as csvfile:
                    #         writer = csv.writer(csvfile,lineterminator='\n')
                    #         for p in self.m5802_list:
                    #             writer.writerow([p])
            elif cmd_msg[0] == "M5803":
                #self.nprintf("Calibration board: %s"%(msg,))
                # raw_params = { part[0]: part[1:].strip()
                #                     for part in cmd_msg }
                raw_msg =  cmd_msg[1][1:].strip()         
                raw_dat = raw_msg.split(':')
                for dat in raw_dat:
                    self.raw_dat_list.append(int(dat))
                self.raw_dat_ptr = self.raw_dat_ptr + 1
                self.nprintf("p%d %s"%(self.raw_dat_ptr,msg))

                pass
            elif cmd_msg[0] == "M5801":
                self.nprintf(msg)
                cmd_params = { part[0]: part[1:].strip()
                        for part in cmd_msg }
                if 'H' in cmd_params:
                    head_ptr = int(cmd_params['H'])
                else:
                    head_ptr = 1
                if head_ptr == 0:
                    headname = "left"
                else:
                    headname = "right"
                if 'P' in cmd_params:
                    val = int(cmd_params['P'])
                    if val == 3:
                        if 'S' in cmd_params:
                            sval = int(cmd_params['S'])
                            if sval == 88:
                                #check whether if the data is ok
                                x_data_is_ok = True
                                y_data_is_ok = True
                                self.nprintf("lets jest check it")
                                p1 = self.m5802_list[1]
                                p2 = self.m5802_list[2]
                                p3 = self.m5802_list[3]
                                self.nprintf("check it %s %s %s "%(str(p1[3]),str(p2[3]),str(p3[3])))
                                if float(p1[2]) == 0 and float(p2[2]) == 0 and float(p3[2]) == 0:
                                    x_data_is_ok = False
                                if float(p1[3]) == 0 and float(p2[3]) == 0 and float(p3[3]) == 0:
                                    y_data_is_ok = False
                                if x_data_is_ok == False and y_data_is_ok == True:
                                    if self.retry_count < 2:
                                        self.retry_count = self.retry_count + 1
                                        self.nprintf("X psd data is bad! try calibrate again!")
                                        self.record_msg_from_ghead("x psd data is bad! try calibrate again!")
                                        self.printer.send_event("ghead:response","messure_reshake_x")
                                        return
                                    else:
                                        self.nprintf("can't get right psd data please check if calibrate board is ok")
                                        self.retry_count = 0 
                                elif y_data_is_ok == False and x_data_is_ok == True:
                                    if self.retry_count < 2:
                                        self.retry_count = self.retry_count + 1
                                        self.nprintf("Y psd data is bad! try calibrate again!")
                                        self.record_msg_from_ghead("y psd data is bad! try calibrate again!")
                                        self.printer.send_event("ghead:response","messure_reshake_y")
                                        return
                                    else:
                                        self.nprintf("can't get right psd data please check if calibrate board is ok")
                                        self.retry_count = 0
                                elif y_data_is_ok == False and x_data_is_ok == False:
                                    if self.retry_count < 2:
                                        self.retry_count = self.retry_count + 1
                                        self.nprintf("XY psd data is all bad! try calibrate again!")
                                        self.record_msg_from_ghead("xy psd data is all bad! try calibrate again!")
                                        self.printer.send_event("ghead:response","messure_reshake_xy")
                                        return
                                    else:
                                        self.nprintf("can't get right psd data please check if calibrate board is ok")
                                        self.retry_count = 0
                                file_name = "/opt/Raise3D/file_{}.csv".format(int(time.time()))
                                with open(file_name, "w") as csvfile:
                                    writer = csv.writer(csvfile,lineterminator='\n')
                                    self.nprintf("start save dat to %s"%file_name)
                                    writer.writerow(("freq","psd_x","psd_y"))
                                    for p in self.m5802_list:
                                        writer.writerow((p[1],p[2],p[3]))
                                csvfile.close()
                                self.m5802_list = []
                                self.nprintf("psd tranmit complete,file name is %s"%file_name)
                                self.printer.send_event("ghead:response","run_to_end")
                            elif sval == -1:
                                self.nprintf("transmit failed,lost communication with Calibration board")

                    elif val == 0:
                        sval = int(cmd_params['S'])
                        if sval == 0 or sval == 1:
                            self.cancel_wait_event()
                            self.nprintf("stop messure adxl345 on%s"%headname)
                            if self.check_version == 0:
                                self.printer.send_event("ghead:response","messure_stop")
                        elif sval == -1:
                            self.nprintf("lose connection with adxl345 on%s"%headname)
                        
                        
                    elif val == 1:
                        sval = int(cmd_params['S'])
                        if sval == 0 or sval == 1: 
                            self.nprintf("start messure adxl345 on%s"%headname)
                            self.cancel_wait_event()
                            if self.check_version == 0:
                                self.printer.send_event("ghead:response","messure_start")
                        elif sval == -1:
                            self.nprintf("lose connection with adxl345 on%s"%headname)
                        
                        
                    elif val == 5:
                        self.nprintf("Calibration board %s : %s"%(headname,msg,))
                    
                    elif val == 4:
                        if 'S' in cmd_params:
                            sval = int(cmd_params['S'])
                            if sval == 1:
                                self.nprintf("Calibration board on %s is in"%headname)
                            else:
                                self.nprintf("Calibration board on %s is not in"%headname)
                    elif val == 2:
                        if 'S' in cmd_params:
                            sval = int(cmd_params['S'])
                            self.psd_num = sval
                elif 'C' in cmd_params:
                        try:
                            val = int(cmd_params['C'])
                            if val == 1:
                                sval = int(cmd_params['S'])
                                if sval == 8:
                                    self.nprintf("complete caculate on %s"%headname) 
                                    self.set_normal_ghead_ctrl(ord('P')+ord('R'),0,5)
                                elif sval == 9:
                                    self.nprintf("retry to caculate on %s"%headname)
                                    if self.check_version == 0:
                                        self.nprintf("notify resonace..")
                                        self.printer.send_event("ghead:response","caculate_fail_event")
                                elif sval == 2:
                                    self.cancel_wait_event()
                                    self.nprintf("Calibration board on the %s is caculating"%headname)
                                    self.printer.send_event("ghead:response","caculating")
                        except:
                            pass                    
                elif 'R' in cmd_params:
                    sval = int(cmd_params['S'])
                    if sval == 88:
                        file_name = "/opt/Raise3D/file_raw_{}.csv".format(int(time.time()))
                        with open(file_name, "w") as csvfile:
                            writer = csv.writer(csvfile,lineterminator='\n')
                            self.nprintf("start save raw dat to %s"%file_name)
                            writer.writerow(("raw_dat",))
                            for p in self.raw_dat_list:
                                writer.writerow((p,float(p)*38.2459))
                        csvfile.close()
                        self.raw_dat_list = []
                        self.raw_dat_ptr = 0
                        self.nprintf("raw dat transmit complete,file name is %s"%file_name)
                    elif sval == -2 :
                        self.raw_dat_ptr = 0
                        self.nprintf("fail to tramsmit raw dat %s"%file_name)
                else:
                    self.nprintf("fetet:" + msg)
                    pass
                pass
        # data = bytearray(params['buff'])
        # lens = int(params['len'])
        # self.nprintf("dat --> %s"%str(data))
        # self.nprintf("len = %d"%lens)
        pass

    def left_location_handle(self,eventtime):
        sta = 1
        if self._location_s['left']:
            sta = 0
        else:
            sta = 1
            self.printer.send_event("hotend:missing","left")
        self.gcode.respond_raw("Heater status: H0 A%d"%(sta))
        return self.reactor.NEVER

    def right_location_handle(self,eventtime):
        sta = 1
        if self._location_s['right']:
            sta = 0
        else:
            sta = 1
            self.printer.send_event("hotend:missing","right")
        self.gcode.respond_raw("Heater status: H1 A%d"%(sta))
        return self.reactor.NEVER

    def handle_respond(self,params):
        if self.printer_type == "Pro3":
            ptr = int(params['gh_ptr'])
            mode = int(params['mode'])
            if mode == ord('R'):
                self.nprintf("movement start ok!!!!!!!!!!!!!!!..")
                self.ghead.set_normal_ghead_ctrl(mode = ord('L'),hs = 0)
                self.gcode.run_script_from_command("M5100 G0 SCAN")
            elif mode == (ord('T') + ord('P')):
                if ptr == 0 :
                    val = int(params['val'])
                    if val == 2:
                        self.nprintf("left hotend temp error")
                elif ptr == 1:
                    val = int(params['val'])
                    if val == 2:
                        self.nprintf("right hotend temp error")
                pass
            elif mode == ord('A'):
                # check hotend is in
                val = int(params['val'])
                self.first_poweron_handle()
                self.gcode.respond_raw("M5100 H%d A%d"%(ptr,val))
                if ptr == 0:
                    hotend = "left"
                else:
                    hotend = "right"
                eventtime = self.reactor.monotonic()
                sta = False
                if val == 0:
                    sta = True
                else:
                    sta = False
                # if self._location_s[hotend] != sta:
                self._location_s[hotend] = sta
                if hotend == "left":
                    self.reactor.update_timer(self.left_location_shake_timer,eventtime + 1.)
                elif hotend == "right":
                    self.reactor.update_timer(self.right_location_shake_timer,eventtime + 1.)
            elif mode == ord('H'):
                # check 24vpower is in
                val = int(params['val'])
                self.first_poweron_handle()
                self.gcode.respond_raw("M5100 H%d H%d"%(ptr,val))
                pass
            elif mode == ord('K'):
                val = int(params['val'])
                self.first_poweron_handle()
                self.gcode.respond_raw("Heater status: H%d K%d"%(ptr,val))
                if ptr == 0:
                    hotend = "left"
                else:
                    hotend = "right"
                self.printer.send_event("ghead:hotend",hotend,val)
                pass
            elif mode == ord('W'):
                # check megnet is in
                val = int(params['val'])
                self.gcode.respond_raw("M5100 S0 W%d"%(val,))
                pass
            elif mode == ord('D'):
                # check megnet is in
                val = int(params['val'])
                self.gcode.respond_raw("M5100 T0 D%d"%(val,))
                pass
            elif mode == (ord('Z') + ord('H')):
                val = int(params['val'])
                sub_ptr  = val&0xff
                main_ptr = (val >> 8)&0xffff
                package_ptr = main_ptr*256 + sub_ptr*32
                tranmit_package = self.firmware_buff[package_ptr:package_ptr+32]
                self.set_len_dat_cmd.send((main_ptr,(ord('U')),sub_ptr,list(tranmit_package)))
                pass
            elif mode == (ord('R') + ord('H')):
                val = int(params['val'])
                if val == 1:
                    self.nprintf("moveboard try to reset ghead ")
                elif val == 0:
                    self.nprintf("ghead boad has been reset ")
                    self.nprintf("ghead restart")
                elif val == 3:
                    self.nprintf("ghead hardware has down!!")
                elif val == 4:
                    self.nprintf("mcu ready")
            elif mode == (ord('U') + ord('G')):
                # update ghead response

                val = int(params['val'])
                if ptr == 0:
                    # set ghead bootloader mode
                    # self.set_normal_ghead_ctrl(ord('U')+ord('G'),1,0)
                    pass
                elif ptr == 1:
                    val = int(params['val'])         
                    if val == 1:
                        # has reset to isp mode try to erase
                        #self.set_normal_ghead_ctrl(ord('U')+ord('G'),2,0)
                        pass       
                else:
                    pass
                pass
            elif mode == (ord('E') + ord('T')):
                val = int(params['val'])
                if val == 0:
                    self.nprintf("new temp in filter %d buff"%(ptr))
                else:
                    self.nprintf("heater %d temp jump to %.2f"%(ptr,float(val)/10))
                pass
        elif self.printer_type == "RMF500":
            if params['mode'] == ord('H'):
                self.gcode.respond_raw("ok heat set")
            elif params['mode'] == ord('B'):
                self.gcode.respond_raw("ok baud set")
            elif params['mode'] == ord('C') and 'ptr' in params and 'val' in params:
                ptr = int(params['ptr'])
                if ptr == 0:
                    self.gcode.respond_raw("ok xmode aff set%s"%(str(params['val'])))
                else:
                    self.gcode.respond_raw("ok ymode aff set%s"%(str(params['val'])))
            elif params['mode'] == ord('A'):
                ptr = int(params['ptr'])
                if ptr == 0:
                    self.gcode.respond_raw("ok xmotor aaf = %s"%(str(params['val'])))
                else:
                    self.gcode.respond_raw("ok ymotor aaf = %s"%(str(params['val'])))
            elif params['mode'] == ord('U'):
                ptr = int(params['ptr'])
                if ptr == 0:
                    self.gcode.respond_raw("ok xmotor status = %s"%(str(params['val'])))
                else:
                    self.gcode.respond_raw("ok ymotor status = %s"%(str(params['val'])))
            elif params['mode'] == ord('T'):
                self.gcode.respond_raw("ok probe set = %s"%(str(params['val'])))
            elif params['mode'] == ord('Z'):
                self.gcode.respond_raw("ok z_lock set = %s"%(str(params['val'])))
            else:
                self.gcode.respond_raw("unexpected mode = "%str(params['mode']))
    def handle_ghead_temp(self,params):    
        readclock= self._mcu.clock32_to_clock64(params['clock'])
        readtime = self._mcu.clock_to_print_time(readclock) - COMM_DELAY_OFFSET
        self.first_poweron_handle()
        #sendtime = params['#sent_time']
        #print ("ghead readtime = %0.3f"%self._mcu.clock_to_print_time(readtime))
        if params['gh_ptr'] == 0 and self._tempv['left'] != None:
            self._tempv['left'] = float(params['value'])/10
            # self.record_msg_from_ghead("heat0 %.1f"%self._tempv['left'])
            fuc = self.handletempcallback['left']
            if fuc is not None:
                if self.errorcount[0] == 0:
                    fuc(self._tempv['left'],readtime)
                else:
                    self.nprintf("simulate temp test %d %.1f,real temp = %.1f"%(self.errorcount[0],self.errortemp[0],self._tempv['left']))
                    self.errorcount[0] = self.errorcount[0] - 1
                    fuc(self.errortemp[0],readtime)
        elif params['gh_ptr'] == 1 and self._tempv['right'] != None:
            self._tempv['right'] = float(params['value'])/10
            # self.record_msg_from_ghead("heat1 %.1f"%self._tempv['right'])
            fuc = self.handletempcallback['right']
            if fuc is not None:
                if self.errorcount[1] == 0:
                    fuc(self._tempv['right'],readtime)
                else:
                    self.nprintf("simulate temp test %d %.1f,real temp = %.1f"%(self.errorcount[1],self.errortemp[1],self._tempv['right']))
                    self.errorcount[1] = self.errorcount[1] - 1
                    fuc(self.errortemp[1],readtime)
    def first_poweron_handle(self):
        if self.first_power_on == True and int(self._ghead_in_s['right']) == 1:
            self.first_power_on = False
            toolhead = self.printer.lookup_object("toolhead")
            if toolhead != None and toolhead.get_current_printer_type != None  :
                if toolhead.get_current_printer_type() == "Pro3":
                    self.set_normal_ghead_ctrl(ord('S'),0,0)
                    self.set_normal_ghead_ctrl(ord('S') + ord('T') ,0,0)
                    self.set_normal_ghead_ctrl(mode = ord('L'),hs = 0)
                    self.nprintf("start ghead and mcu")

    def handle_ghead_sensor_s(self,params):
        if params['gh_ptr'] == 0 and self._ksensor_s['left'] != None:
            self._ksensor_s['left'] = params['sensor']
            self.record_msg_from_ghead("heat0 k%d"%self._ksensor_s['left'])
        elif params['gh_ptr'] == 1 and self._ksensor_s['right'] != None:
            self._ksensor_s['right'] = params['sensor']
            #print ("this sensor status is from right ghead %d"%self._ksensor_s['right'])
            self.record_msg_from_ghead("heat1 k%d"%self._ksensor_s['right'])
    def handle_ghead_location_s(self,params):
        self.first_poweron_handle()

    def handle_ghead_in_s(self,params):
        if params['gh_ptr'] == 0 and self._ghead_in_s['left'] != None:
            # self._ghead_in_s['left'] = params['ghead_s']
            # self.nprintf("this ghead status is from left ghead %d"%self._ghead_in_s['left'])
            pass
        elif params['gh_ptr'] == 1 and self._ghead_in_s['right'] != None:
            if self._ghead_in_s['right'] != params['ghead_s'] :
                self.nprintf("ghead status = %d"%self._ghead_in_s['right'])
            self._ghead_in_s['right'] = params['ghead_s']
            if self._ghead_in_s['right'] == 1:
                self.first_poweron_handle()
                        
    def handle_setfan_res(self,params):
        if params['gh_ptr'] == 0 and self.fan_speed['left'] != None:
            self.fan_speed['left'] = params['f_speed']
            #print ("this set fan res is from left ghead %d"%self._ghead_in_s['left'])
        elif params['gh_ptr'] == 1 and self.fan_speed['right'] != None:
            self.fan_speed['right'] = params['f_speed']
            #print ("this set fan res is from right ghead %d"%self._ghead_in_s['left'])
        
class MCU_adc:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._pin = pin_params['pin']
        self._min_sample = self._max_sample = 0.
        self._sample_time = self._report_time = 0.
        self._sample_count = self._range_check_count = 0
        self._report_clock = 0
        self._last_state = (0., 0.)
        self._oid = self._callback = None
        self._mcu.register_config_callback(self._build_config)
        self._inv_max_adc = 0.
    def get_mcu(self):
        return self._mcu
    def setup_minmax(self, sample_time, sample_count,
                     minval=0., maxval=1., range_check_count=0):
        self._sample_time = sample_time
        self._sample_count = sample_count
        self._min_sample = minval
        self._max_sample = maxval
        self._range_check_count = range_check_count
    def setup_adc_callback(self, report_time, callback):
        self._report_time = report_time
        self._callback = callback
    def get_last_value(self):
        return self._last_state
    def _build_config(self):
        if not self._sample_count:
            return
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd("config_analog_in oid=%d pin=%s" % (
            self._oid, self._pin))
        clock = self._mcu.get_query_slot(self._oid)
        sample_ticks = self._mcu.seconds_to_clock(self._sample_time)
        mcu_adc_max = self._mcu.get_constant_float("ADC_MAX")
        max_adc = self._sample_count * mcu_adc_max
        self._inv_max_adc = 1.0 / max_adc
        self._report_clock = self._mcu.seconds_to_clock(self._report_time)
        min_sample = max(0, min(0xffff, int(self._min_sample * max_adc)))
        max_sample = max(0, min(0xffff, int(
            math.ceil(self._max_sample * max_adc))))
        self._mcu.add_config_cmd(
            "query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d"
            " rest_ticks=%d min_value=%d max_value=%d range_check_count=%d" % (
                self._oid, clock, sample_ticks, self._sample_count,
                self._report_clock, min_sample, max_sample,
                self._range_check_count), is_init=True)
        self._mcu.register_response(self._handle_analog_in_state,
                                    "analog_in_state", self._oid)
    def _handle_analog_in_state(self, params):
        last_value = params['value'] * self._inv_max_adc
        # print (str(params['value']))
        next_clock = self._mcu.clock32_to_clock64(params['next_clock'])
        last_read_clock = next_clock - self._report_clock
        last_read_time = self._mcu.clock_to_print_time(last_read_clock)
        self._last_state = (last_value, last_read_time)
        if self._callback is not None:
            self._callback(last_read_time, last_value)

# Class to retry sending of a query command until a given response is received
class RetryAsyncCommand:
    TIMEOUT_TIME = 5.0
    RETRY_TIME = 0.500
    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.reactor = serial.get_reactor()
        self.completion = self.reactor.completion()
        self.min_query_time = self.reactor.monotonic()
        self.serial.register_response(self.handle_callback, name, oid)
    def handle_callback(self, params):
        if params['#sent_time'] >= self.min_query_time:
            self.min_query_time = self.reactor.NEVER
            self.reactor.async_complete(self.completion, params)
    def get_response(self, cmds, cmd_queue, minclock=0, reqclock=0):
        cmd, = cmds
        self.serial.raw_send_wait_ack(cmd, minclock, reqclock, cmd_queue)
        first_query_time = query_time = self.reactor.monotonic()
        while 1:
            params = self.completion.wait(query_time + self.RETRY_TIME)
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            query_time = self.reactor.monotonic()
            if query_time > first_query_time + self.TIMEOUT_TIME:
                self.serial.register_response(None, self.name, self.oid)
                raise serialhdl.error("Timeout on wait for '%s' response"
                                      % (self.name,))
            self.serial.raw_send(cmd, minclock, minclock, cmd_queue)

# Wrapper around query commands
class CommandQueryWrapper:
    def __init__(self, serial, msgformat, respformat, oid=None,
                 cmd_queue=None, is_async=False, error=serialhdl.error):
        self._serial = serial
        self._cmd = serial.get_msgparser().lookup_command(msgformat)
        serial.get_msgparser().lookup_command(respformat)
        self._response = respformat.split()[0]
        self._oid = oid
        self._error = error
        self._xmit_helper = serialhdl.SerialRetryCommand
        if is_async:
            self._xmit_helper = RetryAsyncCommand
        if cmd_queue is None:
            cmd_queue = serial.get_default_command_queue()
        self._cmd_queue = cmd_queue
    def _do_send(self, cmds, minclock, reqclock):
        xh = self._xmit_helper(self._serial, self._response, self._oid)
        reqclock = max(minclock, reqclock)
        try:
            return xh.get_response(cmds, self._cmd_queue, minclock, reqclock)
        except serialhdl.error as e:
            raise self._error(str(e))
    def send(self, data=(), minclock=0, reqclock=0):
        return self._do_send([self._cmd.encode(data)], minclock, reqclock)
    def send_with_preface(self, preface_cmd, preface_data=(), data=(),
                          minclock=0, reqclock=0):
        cmds = [preface_cmd._cmd.encode(preface_data), self._cmd.encode(data)]
        return self._do_send(cmds, minclock, reqclock)

# Wrapper around command sending
class CommandWrapper:
    def __init__(self, serial, msgformat, cmd_queue=None):
        self._serial = serial
        self.msgformat = msgformat
        self._cmd = serial.get_msgparser().lookup_command(msgformat)
        if cmd_queue is None:
            cmd_queue = serial.get_default_command_queue()
        self._cmd_queue = cmd_queue
    def send(self, data=(), minclock=0, reqclock=0):
        
        cmd = self._cmd.encode(data)
        self._serial.raw_send(cmd, minclock, reqclock, self._cmd_queue)
        

class MCU:
    error = error
    def __init__(self, config, clocksync):
        self._printer = printer = config.get_printer()
        self._clocksync = clocksync
        self._reactor = printer.get_reactor()
        self._name = config.get_name()
        if self._name.startswith('mcu '):
            self._name = self._name[4:]
        
        # Serial port
        wp = "mcu '%s': " % (self._name)
        self._serial = serialhdl.SerialReader(self._reactor,printer, warn_prefix=wp)
        self._baud = 0
        self._canbus_iface = None
        canbus_uuid = config.get('canbus_uuid', None)
        if canbus_uuid is not None:
            self._serialport = canbus_uuid
            self._canbus_iface = config.get('canbus_interface', 'can0')
            cbid = self._printer.load_object(config, 'canbus_ids')
            cbid.add_uuid(config, canbus_uuid, self._canbus_iface)
        else:
            port = None
            try:
                sport = os.listdir("/dev/serial/by-id/")
                for dir in sport:
                    if dir.startswith("usb-Klipper"):
                        port = "/dev/serial/by-id/%s"%dir
                        logging.error("read port -->"+port)
                        break 
            except:
                port = None
                logging.error("can't read port ??")
            if port is not None and self._name == 'mcu':
                config.get('serial')
                self._serialport = port
            else:
            	self._serialport = config.get('serial')
            logging.info('cur port = '+self._serialport)
            if not (self._serialport.startswith("/dev/rpmsg_")
                    or self._serialport.startswith("/tmp/klipper_host_")):
                self._baud = config.getint('baud', 250000, minval=2400)
        # Restarts
        restart_methods = [None, 'arduino', 'cheetah', 'command', 'rpi_usb']
        self._restart_method = 'command'
        if self._baud:
            rmethods = {m: m for m in restart_methods}
            self._restart_method = config.getchoice('restart_method',
                                                    rmethods, None)
        self._reset_cmd = self._config_reset_cmd = None
        self._emergency_stop_cmd = None
        self._is_shutdown = self._is_timeout = False
        self._shutdown_clock = 0
        self._shutdown_msg = ""
        # Config building
        printer.lookup_object('pins').register_chip(self._name, self)
        self._oid_count = 0
        self._config_callbacks = []
        self._config_cmds = []
        self._restart_cmds = []
        self._init_cmds = []
        config.deprecate('pin_map')
        self._pin_map = config.get('pin_map', None)
        self._mcu_freq = 0.
        # Move command queuing
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._max_stepper_error = config.getfloat('max_stepper_error', 0.000025,
                                                  minval=0.)
        self._reserved_move_slots = 0
        self._stepqueues = []
        self._steppersync = None
        # Stats
        self._get_status_info = {}
        self._stats_sumsq_base = 0.
        self._mcu_tick_avg = 0.
        self._mcu_tick_stddev = 0.
        self._mcu_tick_awake = 0.
        self.ghead = None
        self.ghead_heat_status = [False,False]
        # Register handlers
        printer.register_event_handler("klippy:connect", self._connect)
        printer.register_event_handler("klippy:mcu_identify",
                                       self._mcu_identify)
        printer.register_event_handler("klippy:shutdown", self._shutdown)
        printer.register_event_handler("klippy:disconnect", self._disconnect)
        printer.register_event_handler("klippy:ready", self._ready_handle)
        printer.register_event_handler("heat:start", self.start_heat_handle)
        printer.register_event_handler("heat:stop", self.stop_heat_handle)
        printer.register_event_handler("stepper_enable:motor_off",self.motor_off)
        #printer.register_event_handler("zlock:ready", self._ready_handle)
        # ghead create
        self.ghead = MCU_ghead(self,2,self._printer,self._reactor)
        self.tmc_motors = {}
    def start_heat_handle(self,ptr):
        if ptr < 2:
            self.ghead.nprintf("try to switch on heater %d"%ptr)
            self.ghead_heat_status[ptr] = True
            self.ghead.start_heat(ptr)
    def stop_heat_handle(self,ptr):
        if ptr < 2:
            if self.ghead_heat_status[ptr] == True:
                self.ghead.nprintf("try to switch off heater %d"%ptr)
                self.ghead_heat_status[ptr] = False
                self.ghead.stop_heat(ptr)
        pass
    def motor_off(self,print_time):
        # toolhead = self._printer.lookup_objects("toolhead")
        # if toolhead != None:
        #     if toolhead.get_current_printer_type() == "RMF500":
        self.ghead.set_normal_ghead_ctrl(ord('Z'),0,0) 
    def _ready_handle(self):
        toolhead = self._printer.lookup_object("toolhead")
        gcode =  self._printer.lookup_object("gcode")
        if toolhead != None and toolhead.get_current_printer_type != None  :
            self.ghead.set_printer_type(toolhead.get_current_printer_type())
            if toolhead.get_current_printer_type() == "RMF500":
                self.ghead.set_normal_ghead_ctrl(ord('Z'),0,0) 
                gcode.run_script_from_command("M5100 G0 T1")
            elif toolhead.get_current_printer_type() == "Pro3":
                # self.ghead.set_normal_ghead_ctrl(mode = ord('R'),hs = 0)
                # self.ghead.nprintf("init movement board pin!")
                # time.sleep(0.2)
                # self.ghead.set_normal_ghead_ctrl(mode = ord('L'),hs = 0)
                # gcode.run_script_from_command("M5100 G0 SCAN")
                pass
    def get_tmc_motors(self):
        return self.tmc_motors
    def add_tmc_motors(self,name,motor_attr):
        if name in self.tmc_motors:
            return
        #print ("%s : %s"%(name,motor_attr.get_name()))
        self.tmc_motors[name] = motor_attr
    # Serial callbacks
    def reset_mcu(self):
        self._serial.try_remote_reset()
    def get_ghead(self):
        return self.ghead
    def debug_msgproto(self):
        print (self._serial.msgparser.messages_by_name)
    def _handle_mcu_stats(self, params):
        count = params['count']
        tick_sum = params['sum']
        c = 1.0 / (count * self._mcu_freq)
        self._mcu_tick_avg = tick_sum * c
        tick_sumsq = params['sumsq'] * self._stats_sumsq_base
        diff = count*tick_sumsq - tick_sum**2
        self._mcu_tick_stddev = c * math.sqrt(max(0., diff))
        self._mcu_tick_awake = tick_sum / self._mcu_freq
    def _handle_shutdown(self, params):
        if self._is_shutdown:
            return
        self._is_shutdown = True
        clock = params.get("clock")
        if clock is not None:
            self._shutdown_clock = self.clock32_to_clock64(clock)
        self._shutdown_msg = msg = params['static_string_id']
        logging.info("MCU '%s' %s: %s\n%s\n%s", self._name, params['#name'],
                     self._shutdown_msg, self._clocksync.dump_debug(),
                     self._serial.dump_debug())
        prefix = "MCU '%s' shutdown: " % (self._name,)
        if params['#name'] == 'is_shutdown':
            prefix = "Previous MCU '%s' shutdown: " % (self._name,)
        self._printer.invoke_async_shutdown(prefix + msg + error_help(msg))
    def _handle_starting(self, params):
        if not self._is_shutdown:
            self._printer.invoke_async_shutdown("MCU '%s' spontaneous restart"
                                                % (self._name,))
    # Connection phase
    def _check_restart(self, reason):
        start_reason = self._printer.get_start_args().get("start_reason")
        if start_reason == 'firmware_restart':
            return
        logging.info("Attempting automated MCU '%s' restart: %s",
                     self._name, reason)
        self._printer.request_exit('firmware_restart')
        self._reactor.pause(self._reactor.monotonic() + 2.000)
        raise error("Attempt MCU '%s' restart failed" % (self._name,))
    def _connect_file(self, pace=False):
        # In a debugging mode.  Open debug output file and read data dictionary
        start_args = self._printer.get_start_args()
        if self._name == 'mcu':
            out_fname = start_args.get('debugoutput')
            dict_fname = start_args.get('dictionary')
        else:
            out_fname = start_args.get('debugoutput') + "-" + self._name
            dict_fname = start_args.get('dictionary_' + self._name)
        outfile = open(out_fname, 'wb')
        dfile = open(dict_fname, 'rb')
        dict_data = dfile.read()
        dfile.close()
        self._serial.connect_file(outfile, dict_data)
        self._clocksync.connect_file(self._serial, pace)
        # Handle pacing
        if not pace:
            def dummy_estimated_print_time(eventtime):
                return 0.
            self.estimated_print_time = dummy_estimated_print_time
    def _send_config(self, prev_crc):
        # Build config commands
        for cb in self._config_callbacks:
            cb()
        self._config_cmds.insert(0, "allocate_oids count=%d"
                                 % (self._oid_count,))
        # Resolve pin names
        mcu_type = self._serial.get_msgparser().get_constant('MCU')
        ppins = self._printer.lookup_object('pins')
        pin_resolver = ppins.get_pin_resolver(self._name)
        if self._pin_map is not None:
            pin_resolver.add_pin_mapping(mcu_type, self._pin_map)
        for cmdlist in (self._config_cmds, self._restart_cmds, self._init_cmds):
            for i, cmd in enumerate(cmdlist):
                cmdlist[i] = pin_resolver.update_command(cmd)
        # Calculate config CRC
        config_crc = zlib.crc32('\n'.join(self._config_cmds)) & 0xffffffff
        self.add_config_cmd("finalize_config crc=%d" % (config_crc,))
        if prev_crc is not None and config_crc != prev_crc:
            self._check_restart("CRC mismatch")
            raise error("MCU '%s' CRC does not match config" % (self._name,))
        # Transmit config messages (if needed)
        self.register_response(self._handle_starting, 'starting')
        try:
            if prev_crc is None:
                logging.info("Sending MCU '%s' printer configuration...",
                             self._name)
                for c in self._config_cmds:
                    self._serial.send(c)
            else:
                for c in self._restart_cmds:
                    self._serial.send(c)
            # Transmit init messages
            for c in self._init_cmds:
                self._serial.send(c)
        except msgproto.enumeration_error as e:
            enum_name, enum_value = e.get_enum_params()
            if enum_name == 'pin':
                # Raise pin name errors as a config error (not a protocol error)
                raise self._printer.config_error(
                    "Pin '%s' is not a valid pin name on mcu '%s'"
                    % (enum_value, self._name))
            raise
    def _send_get_config(self):
        get_config_cmd = self.lookup_query_command(
            "get_config",
            "config is_config=%c crc=%u move_count=%hu is_shutdown=%c")
        if self.is_fileoutput():
            return { 'is_config': 0, 'move_count': 500, 'crc': 0 }
        config_params = get_config_cmd.send()
        if self._is_shutdown:
            raise error("MCU '%s' error during config: %s" % (
                self._name, self._shutdown_msg))
        if config_params['is_shutdown']:
            raise error("Can not update MCU '%s' config as it is shutdown" % (
                self._name,))
        return config_params
    def _log_info(self):
        msgparser = self._serial.get_msgparser()
        message_count = len(msgparser.get_messages())
        version, build_versions = msgparser.get_version_info()
        log_info = [
            "Loaded MCU '%s' %d commands (%s / %s)"
            % (self._name, message_count, version, build_versions),
            "MCU '%s' config: %s" % (self._name, " ".join(
                ["%s=%s" % (k, v) for k, v in self.get_constants().items()]))]
        return "\n".join(log_info)
    def _connect(self):
        config_params = self._send_get_config()
        # print ("try to connect mcu config is  ".join(config_params))
        if not config_params['is_config']:
            if self._restart_method == 'rpi_usb':
                # Only configure mcu after usb power reset
                self._check_restart("full reset before config")
            # Not configured - send config and issue get_config again
            self._send_config(None)
            config_params = self._send_get_config()
            if not config_params['is_config'] and not self.is_fileoutput():
                raise error("Unable to configure MCU '%s'" % (self._name,))
        else:
            start_reason = self._printer.get_start_args().get("start_reason")
            if start_reason == 'firmware_restart':
                raise error("Failed automated reset of MCU '%s'"
                            % (self._name,))
            # Already configured - send init commands
            self._send_config(config_params['crc'])
        # Setup steppersync with the move_count returned by get_config
        move_count = config_params['move_count']
        if move_count < self._reserved_move_slots:
            raise error("Too few moves available on MCU '%s'" % (self._name,))
        ffi_main, ffi_lib = chelper.get_ffi()
        self._steppersync = ffi_main.gc(
            ffi_lib.steppersync_alloc(self._serial.serialqueue,
                                      self._stepqueues, len(self._stepqueues),
                                      move_count-self._reserved_move_slots),
            ffi_lib.steppersync_free)
        ffi_lib.steppersync_set_time(self._steppersync, 0., self._mcu_freq)
        # Log config information
        move_msg = "Configured MCU '%s' (%d moves)" % (self._name, move_count)
        logging.info(move_msg)
        log_info = self._log_info() + "\n" + move_msg
        self._printer.set_rollover_info(self._name, log_info, log=False)
    def _mcu_identify(self):
        if self.is_fileoutput():
            self._connect_file()
        else:
            resmeth = self._restart_method
            if resmeth == 'rpi_usb' and not os.path.exists(self._serialport):
                # Try toggling usb power
                self._check_restart("enable power")
            try:
                if self._canbus_iface is not None:
                    cbid = self._printer.lookup_object('canbus_ids')
                    nodeid = cbid.get_nodeid(self._serialport)
                    self._serial.connect_canbus(self._serialport, nodeid,
                                                self._canbus_iface)
                elif self._baud:
                    # Cheetah boards require RTS to be deasserted
                    # else a reset will trigger the built-in bootloader.
                    rts = (resmeth != "cheetah")
                    logging.info("connecting  port>>>>>>>" + self._serialport)
                    self._serial.connect_uart(self._serialport, self._baud, rts)
                else:
                    logging.info("connecting pipe>>>>>>>" + self._serialport)
                    self._serial.connect_pipe(self._serialport)
                    
                self._clocksync.connect(self._serial)
            except serialhdl.error as e:
                raise error(str(e))
        logging.info(self._log_info())
        ppins = self._printer.lookup_object('pins')
        pin_resolver = ppins.get_pin_resolver(self._name)
        for cname, value in self.get_constants().items():
            if cname.startswith("RESERVE_PINS_"):
                for pin in value.split(','):
                    pin_resolver.reserve_pin(pin, cname[13:])
        self._mcu_freq = self.get_constant_float('CLOCK_FREQ')
        self._stats_sumsq_base = self.get_constant_float('STATS_SUMSQ_BASE')
        self._emergency_stop_cmd = self.lookup_command("emergency_stop")
        self._reset_cmd = self.try_lookup_command("reset")
        self._config_reset_cmd = self.try_lookup_command("config_reset")
        ext_only = self._reset_cmd is None and self._config_reset_cmd is None
        msgparser = self._serial.get_msgparser()
        mbaud = msgparser.get_constant('SERIAL_BAUD', None)
        if self._restart_method is None and mbaud is None and not ext_only:
            self._restart_method = 'command'
        version, build_versions = msgparser.get_version_info()
        self._get_status_info['mcu_version'] = version
        self._get_status_info['mcu_build_versions'] = build_versions
        self._get_status_info['mcu_constants'] = msgparser.get_constants()
        self.register_response(self._handle_shutdown, 'shutdown')
        self.register_response(self._handle_shutdown, 'is_shutdown')
        self.register_response(self._handle_mcu_stats, 'stats')
        if self._name == 'mcu':
            self.ghead.ghead_start(2)
        #ptr = (0,50)
        #self.ghead.start_ghead_moitor.send(ptr)
    # Config creation helpers
    def setup_pin(self, pin_type, pin_params):
        pcs = {'endstop': MCU_endstop,
               'digital_out': MCU_digital_out, 'pwm': MCU_pwm, 'adc': MCU_adc}
        if pin_type not in pcs:
            raise pins.error("pin type %s not supported on mcu" % (pin_type,))
        return pcs[pin_type](self, pin_params)
    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1
    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        if is_init:
            self._init_cmds.append(cmd)
        elif on_restart:
            self._restart_cmds.append(cmd)
        else:
            self._config_cmds.append(cmd)
    def get_query_slot(self, oid):
        slot = self.seconds_to_clock(oid * .01)
        t = int(self.estimated_print_time(self._reactor.monotonic()) + 1.5)
        return self.print_time_to_clock(t) + slot
    def register_stepqueue(self, stepqueue):
        self._stepqueues.append(stepqueue)
    def request_move_queue_slot(self):
        self._reserved_move_slots += 1
    def seconds_to_clock(self, time):
        return int(time * self._mcu_freq)
    def get_max_stepper_error(self):
        return self._max_stepper_error
    # Wrapper functions
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def register_response(self, cb, msg, oid=None):
        self._serial.register_response(cb, msg, oid)
    def alloc_command_queue(self):
        return self._serial.alloc_command_queue()
    def lookup_command(self, msgformat, cq=None):
        return CommandWrapper(self._serial, msgformat, cq)
    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, is_async=False):
        return CommandQueryWrapper(self._serial, msgformat, respformat, oid,
                                   cq, is_async, self._printer.command_error)
    def try_lookup_command(self, msgformat):
        try:
            return self.lookup_command(msgformat)
        except self._serial.get_msgparser().error as e:
            return None
    def lookup_command_tag(self, msgformat):
        all_msgs = self._serial.get_msgparser().get_messages()
        return {fmt: msgtag for msgtag, msgtype, fmt in all_msgs}[msgformat]
    def get_enumerations(self):
        return self._serial.get_msgparser().get_enumerations()
    def get_constants(self):
        return self._serial.get_msgparser().get_constants()
    def get_constant_float(self, name):
        return self._serial.get_msgparser().get_constant_float(name)
    def print_time_to_clock(self, print_time):
        return self._clocksync.print_time_to_clock(print_time)
    def clock_to_print_time(self, clock):
        return self._clocksync.clock_to_print_time(clock)
    def estimated_print_time(self, eventtime):
        return self._clocksync.estimated_print_time(eventtime)
    def clock32_to_clock64(self, clock32):
        return self._clocksync.clock32_to_clock64(clock32)
    # Restarts
    def _disconnect(self):
        self._serial.disconnect()
        self._steppersync = None
    def _shutdown(self, force=False):
        if (self._emergency_stop_cmd is None
            or (self._is_shutdown and not force)):
            return
        self._emergency_stop_cmd.send()
    def _restart_arduino(self):
        logging.info("Attempting MCU '%s' reset", self._name)
        self._disconnect()
        serialhdl.arduino_reset(self._serialport, self._reactor)
    def _restart_cheetah(self):
        logging.info("Attempting MCU '%s' Cheetah-style reset", self._name)
        self._disconnect()
        serialhdl.cheetah_reset(self._serialport, self._reactor)
    def _restart_via_command(self):
        if ((self._reset_cmd is None and self._config_reset_cmd is None)
            or not self._clocksync.is_active()):
            logging.info("Unable to issue reset command on MCU '%s'",
                         self._name)
            return
        if self._reset_cmd is None:
            # Attempt reset via config_reset command
            logging.info("Attempting MCU '%s' config_reset command", self._name)
            self._is_shutdown = True
            self._shutdown(force=True)
            self._reactor.pause(self._reactor.monotonic() + 0.015)
            self._config_reset_cmd.send()
        else:
            # Attempt reset via reset command
            logging.info("Attempting MCU '%s' reset command", self._name)
            self._reset_cmd.send()
        self._reactor.pause(self._reactor.monotonic() + 0.015)
        self._disconnect()
    def _restart_rpi_usb(self):
        logging.info("Attempting MCU '%s' reset via rpi usb power", self._name)
        self._disconnect()
        chelper.run_hub_ctrl(0)
        self._reactor.pause(self._reactor.monotonic() + 2.)
        chelper.run_hub_ctrl(1)
    def microcontroller_restart(self):
        if self._restart_method == 'rpi_usb':
            self._restart_rpi_usb()
        elif self._restart_method == 'command':
            self._restart_via_command()
        elif self._restart_method == 'cheetah':
            self._restart_cheetah()
        else:
            self._restart_arduino()
    # Misc external commands
    def is_fileoutput(self):
        return self._printer.get_start_args().get('debugoutput') is not None
    def is_shutdown(self):
        return self._is_shutdown
    def get_shutdown_clock(self):
        return self._shutdown_clock
    def flush_moves(self, print_time):
        if self._steppersync is None:
            return
        clock = self.print_time_to_clock(print_time)
        if clock < 0:
            return
        ret = self._ffi_lib.steppersync_flush(self._steppersync, clock)
        if ret:
            raise error("Internal error in MCU '%s' stepcompress"
                        % (self._name,))
    def check_active(self, print_time, eventtime):
        if self._steppersync is None:
            return
        offset, freq = self._clocksync.calibrate_clock(print_time, eventtime)
        self._ffi_lib.steppersync_set_time(self._steppersync, offset, freq)
        if (self._clocksync.is_active() or self.is_fileoutput()
            or self._is_timeout):
            return
        self._is_timeout = True
        logging.info("Timeout with MCU '%s' (eventtime=%f)",
                     self._name, eventtime)
        self._printer.invoke_shutdown("Lost communication with MCU '%s'" % (
            self._name,))
    def get_status(self, eventtime=None):
        return dict(self._get_status_info)
    def stats(self, eventtime):
        load = "mcu_awake=%.03f mcu_task_avg=%.06f mcu_task_stddev=%.06f" % (
            self._mcu_tick_awake, self._mcu_tick_avg, self._mcu_tick_stddev)
        stats = ' '.join([load, self._serial.stats(eventtime),
                          self._clocksync.stats(eventtime)])
        parts = [s.split('=', 1) for s in stats.split()]
        last_stats = {k:(float(v) if '.' in v else int(v)) for k, v in parts}
        self._get_status_info['last_stats'] = last_stats
        return False, '%s: %s' % (self._name, stats)

Common_MCU_errors = {
    ("Timer too close",): """
This often indicates the host computer is overloaded. Check
for other processes consuming excessive CPU time, high swap
usage, disk errors, overheating, unstable voltage, or
similar system problems on the host computer.""",
    ("Missed scheduling of next ",): """
This is generally indicative of an intermittent
communication failure between micro-controller and host.""",
    ("ADC out of range",): """
This generally occurs when a heater temperature exceeds
its configured min_temp or max_temp.""",
    ("Rescheduled timer in the past", "Stepper too far in past"): """
This generally occurs when the micro-controller has been
requested to step at a rate higher than it is capable of
obtaining.""",
    ("Command request",): """
This generally occurs in response to an M112 G-Code command
or in response to an internal error in the host software.""",
}

def error_help(msg):
    for prefixes, help_msg in Common_MCU_errors.items():
        for prefix in prefixes:
            if msg.startswith(prefix):
                return help_msg
    return ""

def add_printer_objects(config):
    printer = config.get_printer()
    reactor = printer.get_reactor()
    mainsync = clocksync.ClockSync(reactor)
    printer.add_object('mcu', MCU(config.getsection('mcu'), mainsync))
    for s in config.get_prefix_sections('mcu '):
        printer.add_object(s.section, MCU(
            s, clocksync.SecondarySync(reactor, mainsync)))

def get_printer_mcu(printer, name):
    if name == 'mcu':
        return printer.lookup_object(name)
    return printer.lookup_object('mcu ' + name)
