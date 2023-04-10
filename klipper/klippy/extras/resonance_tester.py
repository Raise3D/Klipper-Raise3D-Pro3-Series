# A utility class to test resonances of the printer
#
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from datetime import datetime
import logging, math, os, time
from . import shaper_calibrate

class TestAxis:
    def __init__(self, axis=None, vib_dir=None):
        if axis is None:
            self._name = "axis=%.3f,%.3f" % (vib_dir[0], vib_dir[1])
        else:
            self._name = axis
        if vib_dir is None:
            self._vib_dir = (1., 0.) if axis == 'x' else (0., 1.)
        else:
            s = math.sqrt(sum([d*d for d in vib_dir]))
            self._vib_dir = [d / s for d in vib_dir]
    def matches(self, chip_axis):
        if self._vib_dir[0] and 'x' in chip_axis:
            return True
        if self._vib_dir[1] and 'y' in chip_axis:
            return True
        return False
    def get_name(self):
        return self._name
    def get_point(self, l):
        return (self._vib_dir[0] * l, self._vib_dir[1] * l)

def _parse_axis(gcmd, raw_axis):
    if raw_axis is None:
        return None
    raw_axis = raw_axis.lower()
    if raw_axis in ['x', 'y']:
        return TestAxis(axis=raw_axis)
    dirs = raw_axis.split(',')
    if len(dirs) != 2:
        raise gcmd.error("Invalid format of axis '%s'" % (raw_axis,))
    try:
        dir_x = float(dirs[0].strip())
        dir_y = float(dirs[1].strip())
    except:
        raise gcmd.error(
                "Unable to parse axis direction '%s'" % (raw_axis,))
    return TestAxis(vib_dir=(dir_x, dir_y))

class VibrationPulseTest:
    def __init__(self, config,resonance_test = None):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.min_freq = config.getfloat('min_freq', 5, minval=1.)
        # Defaults are such that max_freq * accel_per_hz == 10000 (max_accel)
        self.max_freq = config.getfloat('max_freq', 10000. / 75.,
                                        minval=self.min_freq, maxval=200.)
        self.accel_per_hz = config.getfloat('accel_per_hz', 75., above=0.)
        self.hz_per_sec = config.getfloat('hz_per_sec', 1.,
                                          minval=0.1, maxval=2.)

        self.probe_points = config.getlists('probe_points', seps=(',', '\n'),
                                            parser=float, count=3)
        self.resonance_test = resonance_test                          
    def get_start_test_points(self):
        return self.probe_points
    def get_cur_height(self):
        point = list(self.probe_points[0])
        return point[2]
    def set_start_point(self,ptr = 2,val = 30):
        point = list(self.probe_points[0])
        print (point)
        if ptr == 0:
            point[0] = val
        elif ptr == 1:
            point[1] = val
        elif ptr == 2:
            point[2] = val   
        else:
            print ("wrong point ptr..")
        self.probe_points = (tuple(point),)
    def prepare_test(self, gcmd):
        self.freq_start = gcmd.get_float("FREQ_START", self.min_freq, minval=1.)
        self.freq_end = gcmd.get_float("FREQ_END", self.max_freq,
                                       minval=self.freq_start, maxval=200.)
        self.hz_per_sec = gcmd.get_float("HZ_PER_SEC", self.hz_per_sec,
                                         above=0., maxval=2.)
    def run_test(self, axis, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        mcu = self.printer.check_object("mcu")
        X, Y, Z, E = toolhead.get_position()
        sign = 1.
        freq = self.freq_start
        # Override maximum acceleration and acceleration to
        # deceleration based on the maximum test frequency
        systime = self.printer.get_reactor().monotonic()
        toolhead_info = toolhead.get_status(systime)
        old_max_accel = toolhead_info['max_accel']
        old_max_accel_to_decel = toolhead_info['max_accel_to_decel']
        max_accel = self.freq_end * self.accel_per_hz
        self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    max_accel, max_accel))
        input_shaper = self.printer.lookup_object('input_shaper', None)
        if input_shaper is not None and not gcmd.get_int('INPUT_SHAPING', 0):
            input_shaper.disable_shaping()
            gcmd.respond_info("Disabled [input_shaper] for resonance testing")
        else:
            input_shaper = None
        gcmd.respond_info("Testing frequency %.0f Hz" % (freq,))
        # self.resonance_test.start_adxl_messure()
        # gcmd.respond_info("start adxl collect.!!!..")
        while freq <= self.freq_end + 0.000001: 
            t_seg = .25 / freq
            accel = self.accel_per_hz * freq
            max_v = accel * t_seg
            toolhead.cmd_M204(self.gcode.create_gcode_command(
                "M204", "M204", {"S": accel}))
            L = 0.5 * accel * t_seg**2
            dX, dY = axis.get_point(L)
            nX = X + sign * dX
            nY = Y + sign * dY
            toolhead.move([nX, nY, Z, E], max_v)
            toolhead.move([X, Y, Z, E], max_v)
            sign = -sign
            old_freq = freq
            freq += 2. * t_seg * self.hz_per_sec
            if math.floor(freq) > math.floor(old_freq):
                gcmd.respond_info("Testing frequency %.2f Hz" % (freq,))
        # Restore the original acceleration values
        self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    old_max_accel, old_max_accel_to_decel))
        # Restore input shaper if it was disabled for resonance testing
        if input_shaper is not None:
            input_shaper.enable_shaping()
            gcmd.respond_info("Re-enabled [input_shaper]")

class ResonanceTester:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.move_speed = config.getfloat('move_speed', 50., above=0.)
        self.test = VibrationPulseTest(config,self)
        self.report_dir = "/media/usb"
        self.target_dir = None
        self.accel_chips = None
        self.reactor = self.printer.get_reactor()
        self.target_folder_name = "%s/reson_test"%self.report_dir
        self.report_m_dir = self.target_folder_name 
        if not os.path.isdir(self.target_folder_name):
            print ("folder : %s is not exist"%self.target_folder_name)
            os.system("mkdir -p %s"%self.target_folder_name)
        self.is_using_board_adxl345 = config.getboolean('using_board_adxl345', False)
        if self.is_using_board_adxl345 == True:
            if not config.get('accel_chip_x', None):
                self.accel_chip_names = [('xy', config.get('accel_chip').strip())]
            else:
                self.accel_chip_names = [
                    ('x', config.get('accel_chip_x').strip()),
                    ('y', config.get('accel_chip_y').strip())]
                if self.accel_chip_names[0][1] == self.accel_chip_names[1][1]:
                    self.accel_chip_names = [('xy', self.accel_chip_names[0][1])]
        else:
            self.accel_chip_names = []
        self.max_smoothing = config.getfloat('max_smoothing', None, minval=0.05)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("MEASURE_AXES_NOISE",
                                    self.cmd_MEASURE_AXES_NOISE,
                                    desc=self.cmd_MEASURE_AXES_NOISE_help)
        self.gcode.register_command("TEST_RESONANCES",
                                    self.cmd_TEST_RESONANCES,
                                    desc=self.cmd_TEST_RESONANCES_help)
        self.gcode.register_command("SHAPER_CALIBRATE",
                                    self.cmd_SHAPER_CALIBRATE,
                                    desc=self.cmd_SHAPER_CALIBRATE_help)
        self.gcode.register_command("set_folder",
                                    self.cmd_set_target_folder,
                                    desc=None)
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.printer.register_event_handler("ghead:response", self.ghead_response_handle)
        self.gcode.register_command('M5800',self.set_start_point,desc = None)
        self.gcode.register_command('M5801',self.set_resonace_collect,desc = None)
        self.cur_height = self.test.get_cur_height()
        self.cur_pos = 1
        self.cur_axes = 'X'
        self.cur_axes_ptr = 0
        self.need_vibrate_all = False
        self.recalibrate_x = False
        self.set_y_axes = 0
        self.cmd_handle_timer = self.reactor.register_timer(
                self.cmd_handle, self.reactor.NEVER)
        self.cmd_consent = ""
        self.recaculate_count = 0
        mcu = self.printer.check_object("mcu")
        self.ghead = mcu.get_ghead()
        self.run_status = "idle"
    def cmd_handle(self,eventtime):
        self.gcode.run_script_from_command(self.cmd_consent)
        return self.reactor.NEVER
    def start_send_cmd(self,cmd):
        self.cmd_consent = cmd
        eventtime = self.reactor.monotonic()
        self.reactor.update_timer(self.cmd_handle_timer,eventtime + 0.1)
    def ghead_response_handle(self,event):
        if event == "messure_start":
            if self.run_status == "wait_start_messure":
                self.run_status = "messure_start"
                self.gcode.respond_raw("algrithm board is start measure,just valibrate %s"%self.cur_axes)
                #self.gcode.run_script_from_command("TEST_RESONANCES axis=%s"%self.cur_axes)
                self.start_send_cmd("TEST_RESONANCES axis=%s"%self.cur_axes)
            elif self.run_status == "vibrating":
                self.gcode.respond_raw("klipper is doing resonace testing")
            else:
                pass
        elif event == "messure_reshake_y":
            self.cur_axes = 'Y'
            self.need_vibrate_all = True
            self.gcode.respond_raw("because Y dat is bad so just recalibrate Y axes")
            self.run_status = "wait_start_messure"
            self.ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,self.package_collect_ifo(0,1,1))
            self.ghead.response_wait("messure_start_response",ord('P')+ord('R'),self.package_collect_ifo(0,1,1),0)
            pass
        elif event == "messure_reshake_xy":
            self.cur_axes = 'X'
            self.recalibrate_x = False
            self.need_vibrate_all == True
            self.gcode.respond_raw("because XY dat is all bad so just recalibrate all")
            self.run_status = "wait_start_messure"
            self.ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,self.package_collect_ifo(0,0,1))
            self.ghead.response_wait("messure_start_response",ord('P')+ord('R'),self.package_collect_ifo(0,0,1),0)
            pass
        elif event == "messure_reshake_x":
            self.cur_axes = 'X'
            self.recalibrate_x = True
            self.need_vibrate_all == False
            self.gcode.respond_raw("because X dat is bad so just recalibrate x axes")
            self.run_status = "wait_start_messure"
            self.ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,self.package_collect_ifo(0,0,1))
            self.ghead.response_wait("messure_start_response",ord('P')+ord('R'),self.package_collect_ifo(0,0,1),0)
            pass
        elif event == "caculate_fail_event":
            if self.recaculate_count < 3:
                if self.run_status == "do_caculating":
                    self.gcode.respond_raw("restart caculate count = %d"%self.recaculate_count)
                    self.recaculate_count = self.recaculate_count +1
                    newv = 0
                    newv = newv | (2 << 8)
                    newv = newv | 3
                    #ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,newv)
                    #self.gcode.respond_raw("start caculate = %d"%newv)
                    #ghead.response_wait("caculate_start_response",ord('P')+ord('R'),newv,0)
                    self.run_status = "wait_caculating"
                    self.start_send_cmd("M5801 C1 S2")
            elif self.recaculate_count == 3:
                self.run_status = "idle"
                self.gcode.respond_raw("fail to caculate totally after 3 times retry!!!")
            pass
        elif event == "caculating":
            self.run_status = "do_caculating"
            pass
        elif event == "transmiting":
            self.run_status = "transmiting"
            pass
        elif event == "run_to_end":
            self.run_status = "idle"
            self.gcode.respond_raw("complete resonance vibrate test process!")
        elif event == "messure_stop":
            mcu = self.printer.check_object("mcu")
            if mcu == None:
                self.gcode.respond_error("can't find mcu object!!!!")
                return  
            else:
                ghead = mcu.get_ghead()
            if self.need_vibrate_all == True and self.cur_axes == 'X' and self.recalibrate_x == False:
                if self.run_status == "end_vibrating":
                    self.cur_axes = 'Y'
                    self.run_status = "wait_start_messure"
                    self.gcode.respond_raw("now try to valibrate %s axes!!"%self.cur_axes)
                    ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,self.package_collect_ifo(0,1,1))
                    # self.gcode.run_script_from_command("TEST_RESONANCES axis=%s"%self.cur_axes)
                    ghead.response_wait("messure_start_response",ord('P')+ord('R'),self.package_collect_ifo(0,1,1),0)
                pass
            elif (self.need_vibrate_all == True and self.cur_axes == 'Y') or self.recalibrate_x == True:
                if self.run_status == "end_vibrating":
                    self.need_vibrate_all == False
                    self.recalibrate_x = False
                    self.recaculate_count = 0
                    newv = 0
                    newv = newv | (2 << 8)
                    newv = newv | 3
                    self.gcode.respond_raw("ok complete all valibrate...")
                    self.run_status = "wait_caculating"
                    self.gcode.respond_raw("start caculate = %d"%newv)
                    ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,newv)
                    ghead.response_wait("caculate_start_response",ord('P')+ord('R'),newv,0)
        pass
    def start_adxl_messure(self):
        if self.ghead != None:
            self.ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,self.package_collect_ifo(0,self.cur_axes_ptr,1))
    def set_resonace_collect(self,gcmd):
        cmdlist = gcmd._params
        mcu = self.printer.check_object("mcu")
        if mcu == None:
            gcmd.respond_error("can't find mcu object!!!!")
            return  
        else:
            ghead = mcu.get_ghead()
        if 'P' in cmdlist:
            val = int(cmdlist['P'])
            if val == 1:
                # start messure
                axes = gcmd.get_int('S',0)
                fre  = gcmd.get_int('F',3200)
                fren = fre/400
                newv = fren
                newv = newv<<16
                newv = newv | (axes << 8)
                newv = newv | 1
                ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,newv)
                gcmd.respond_raw("new v = %d"%newv)
                pass
            elif val == 0:
                # stop messure
                ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,2)
                pass
            elif val == 2:
                # try to get messure num
                ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,4)
                pass
            elif val == 3:
                # start transmit
                ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,5)
                pass
            elif val == 4:
                # check Algorithm board
                ghead.transmit_cmd_to_ghead("M5801 P4")
                # ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,0)
            elif val == 5:
                # read adxl345
                ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,6)
            elif val == 6:
                # read adxl345
                x_axes = gcmd.get_int('X',None)
                y_axes  = gcmd.get_int('Y',None)
                z_axes  = gcmd.get_int('Z',None)
                if x_axes == None or y_axes == None or z_axes == None:
                    gcmd.respond_error("lose some axes when do map")
                    return 
                if x_axes > 2 or y_axes > 2 or z_axes > 2:
                    gcmd.respond_error("axes num shouldn't exeed 2")
                    return 
                if x_axes == y_axes or x_axes == z_axes or y_axes == z_axes:
                    gcmd.respond_error("axes num shouldn't be same")
                    return 
                newv = x_axes<<24 | y_axes<<16 | z_axes <<8 | 7
                ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,newv)
            else:
                ghead.transmit_cmd_to_ghead("M5801 P%d"%val)
        elif 'C' in cmdlist:
            val = int(cmdlist['C'])
            if val == 1:
                axes = gcmd.get_int('S',0)
                newv = 0
                newv = newv | (axes << 8)
                newv = newv | 3
                gcmd.respond_raw("start caculate = %d"%newv)
                self.run_status = "wait_caculate_response"
                ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,newv)
                ghead.response_wait("caculate_start_response",ord('P')+ord('R'),newv,0)
        else:
            cmdline = str(gcmd._commandline)
            m5801_cmd = list(cmdline)
            ghead.set_len_dat_cmd.send((0,(ord('G')+ord('H')),0,m5801_cmd))
        pass
    def package_collect_ifo(self,fre,axes,cmd):
        newv = fre
        newv = newv<<16
        newv = newv | (axes << 8)
        newv = newv | cmd
        return newv
    def set_start_point(self,gcmd):
        cmdlist = gcmd._params
        gmove_cmd = "G1"
        if "Z" in cmdlist :
            val = float(cmdlist["Z"])
            if val >=0 and val <= 350:
                print ("before point is",self.test.get_start_test_points())
                self.test.set_start_point(2,val)
                # self.gcode.run_script_from_command("G1 Z%.3f"%val)
                gmove_cmd = "%s Z%.3f"%(gmove_cmd,val)
            
                print ("cur point is",self.test.get_start_test_points())
        if "X" in cmdlist :
            val = int(cmdlist["X"])
            if val >=0 and val <= 400:
                # self.gcode.run_script_from_command("G1 X%.3f"%val)
                gmove_cmd = "%s X%.3f"%(gmove_cmd,val)
                self.test.set_start_point(0,val)
        if "Y" in cmdlist :
            val = int(cmdlist["Y"])
            if val >=0 and val <= 400:
                self.test.set_start_point(1,val)
                gmove_cmd = "%s Y%.3f"%(gmove_cmd,val)
        if 'P' in cmdlist:
            val = int(cmdlist["P"])
            if self.cur_pos != val :
                time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
                self.report_m_dir = "%s/reson_test/%s"%(self.report_dir,time)
                dir = "%s/reson_test/%s/%d"%(self.report_dir,time,val)
                if not os.path.exists(dir):
                    os.makedirs(dir)
                self.target_dir = dir
                self.cur_pos = val
        if "S" in cmdlist:
            axis_ptr = int(cmdlist['S'])
            axis =  "XY"
            if gmove_cmd != "G1":
                self.gcode.run_script_from_command(gmove_cmd)
                toolhead = self.printer.lookup_object('toolhead')
                toolhead.wait_moves()
            if axis_ptr <= 1:
                axi = axis[axis_ptr]
                self.cur_axes = axi
                gcmd.respond_raw("ok start resonace test!!")
                mcu = self.printer.check_object("mcu")
                if mcu == None:
                    gcmd.respond_error("can't find mcu object!!!!")
                    return
                else:
                    ghead = mcu.get_ghead()
                axes = axis_ptr
                self.cur_axes_ptr = axis_ptr
                fre  = gcmd.get_int('F',3200)
                fren = fre/400
                newv = fren
                newv = newv<<16
                newv = newv | (axes << 8)
                newv = newv | 1
                if ghead.check_version == 0:
                    ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,newv)
                    gcmd.respond_raw("new v = %d"%newv)
                    self.run_status = "wait_start_messure"
                    ghead.response_wait("messure_start_response",ord('P')+ord('R'),newv,0)
                # self.gcode.run_script_from_command("TEST_RESONANCES axis=%s"%axi)
                self.need_vibrate_all = False
                
            elif axis_ptr >= 2:
                self.need_vibrate_all = True
                self.recalibrate_x = False
                self.recalibrate_y = False
                self.cur_axes = 'X'
                gcmd.respond_raw("ok begin automatic vabirate start %s resonace test!!"%self.cur_axes)
                mcu = self.printer.check_object("mcu")
                if mcu == None:
                    gcmd.respond_error("can't find mcu object!!!!")
                    return  
                else:
                    ghead = mcu.get_ghead()
                axes = 0
                fre  = gcmd.get_int('F',3200)
                fren = fre/400
                newv = self.package_collect_ifo(fren,axes,1)
                self.set_y_axes = self.package_collect_ifo(fren,1,1)
                ghead.set_normal_ghead_ctrl((ord('P')+ord('R')),0,newv)
                self.run_status = "wait_start_messure"
                ghead.response_wait("messure_start_response",ord('P')+ord('R'),newv,0)
                if ghead.check_version == 0:
                    gcmd.respond_raw("new v = %d"%newv)
                    #self.gcode.run_script_from_command("TEST_RESONANCES axis=%s"%self.cur_axes)
                
    def connect(self):
        if self.is_using_board_adxl345 == True:
            self.accel_chips = [
                    (chip_axis, self.printer.lookup_object(chip_name))
                    for chip_axis, chip_name in self.accel_chip_names]

    def _run_test(self, gcmd, axes, helper, raw_name_suffix=None):
        toolhead = self.printer.lookup_object('toolhead')
        calibration_data = {axis: None for axis in axes}

        self.test.prepare_test(gcmd)
        test_points = self.test.get_start_test_points()
        for point in test_points:
            toolhead.manual_move(point, self.move_speed)
            if len(test_points) > 1:
                gcmd.respond_info(
                        "Probing point (%.3f, %.3f, %.3f)" % tuple(point))
            for axis in axes:
                toolhead.wait_moves()
                toolhead.dwell(0.500)
                if len(axes) > 1:
                    gcmd.respond_info("Testing axis %s" % axis.get_name())
                if self.is_using_board_adxl345 == True:
                    raw_values = []
                    for chip_axis, chip in self.accel_chips:
                        if axis.matches(chip_axis):
                            aclient = chip.start_internal_client()
                            raw_values.append((chip_axis, aclient))
                # Generate moves
                self.test.run_test(axis, gcmd)
                if self.is_using_board_adxl345 == True:
                    for chip_axis, aclient in raw_values:
                        aclient.finish_measurements()
                        if raw_name_suffix is not None:
                            raw_name = self.get_filename(
                                    'raw_data', raw_name_suffix, axis,
                                    point if len(test_points) > 1 else None)
                            aclient.write_to_file(raw_name)
                            gcmd.respond_info(
                                    "Writing raw accelerometer data to "
                                    "%s file" % (raw_name,))
                    if helper is None:
                        continue
                    for chip_axis, chip_values in raw_values:
                        if not chip_values:
                            raise gcmd.error(
                                    "%s-axis accelerometer measured no data" % (
                                        chip_axis,))
                        new_data = helper.process_accelerometer_data(chip_values)
                        if calibration_data[axis] is None:
                            calibration_data[axis] = new_data
                        else:
                            calibration_data[axis].add_data(new_data)
        if self.is_using_board_adxl345 == True:
            return calibration_data
        return None
    def cmd_set_target_folder(self,gcmd):
        cmdlist = gcmd._params
        time = str(datetime.date())
        if 'P' in cmdlist:
            val = int(cmdlist["P"])
            dir = "%s/reson_test/%s/%d"%(self.report_dir,time,val)
            res = os.system("ls %s >/dev/null"%dir)
            if res != 0 :
                print ("folder : %s is not exist"%dir)
                os.system("mkdir -p %s"%dir)
            self.cur_pos = val
        print (gcmd)
    cmd_TEST_RESONANCES_help = ("Runs the resonance test for a specifed axis")
    def cmd_TEST_RESONANCES(self, gcmd):
        self.run_status = "vibrating"
        # Parse parameters
        axis = _parse_axis(gcmd, gcmd.get("AXIS").lower())
        mcu = self.printer.check_object("mcu")
        ghead = mcu.get_ghead()
        if self.is_using_board_adxl345 == True:
            outputs = gcmd.get("OUTPUT", "resonances").lower().split(',')
            for output in outputs:
                if output not in ['resonances', 'raw_data']:
                    raise gcmd.error("Unsupported output '%s', only 'resonances'"
                                    " and 'raw_data' are supported" % (output,))
            if not outputs:
                raise gcmd.error("No output specified, at least one of 'resonances'"
                                " or 'raw_data' must be set in OUTPUT parameter")
            name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
            if not self.is_valid_name_suffix(name_suffix):
                raise gcmd.error("Invalid NAME parameter")
            csv_output = 'resonances' in outputs
            raw_output = 'raw_data' in outputs
        else:
            csv_output = None
            raw_output = None

        # Setup calculation of resonances
        if csv_output and self.is_using_board_adxl345 == True:
            helper = shaper_calibrate.ShaperCalibrate(self.printer)
        else:
            helper = None
        if self.is_using_board_adxl345 == True:
            data = self._run_test(
                    gcmd, [axis], helper,
                    raw_name_suffix=name_suffix if raw_output else None)[axis]
        else:
            self._run_test(gcmd, [axis], helper)

        self.run_status = "end_vibrating"
        ghead.set_normal_ghead_ctrl(ord('P')+ord('R'),0,2)
        ghead.response_wait("messure_stop_response",ord('P')+ord('R'),2,0)
        gcmd.respond_raw("ok complete resonace %s test!!"%self.cur_axes)
        # if mcu == None:
        #     gcmd.respond_error("can't find mcu object!!!!")
        #     return  
        # else:
            
        if csv_output and self.is_using_board_adxl345 == True:
            csv_name = self.save_calibration_data('resonances', name_suffix,
                                                  helper, axis, data)
            gcmd.respond_info(
                    "Resonances data written to %s file" % (csv_name,))
            gcmd.respond_raw("report_cvs_dir %s"%self.report_m_dir)
    cmd_SHAPER_CALIBRATE_help = (
        "Simular to TEST_RESONANCES but suggest input shaper config")
    def cmd_SHAPER_CALIBRATE(self, gcmd):
        # Parse parameters
        axis = gcmd.get("AXIS", None)
        if not axis:
            calibrate_axes = [TestAxis('x'), TestAxis('y')]
        elif axis.lower() not in 'xy':
            raise gcmd.error("Unsupported axis '%s'" % (axis,))
        else:
            calibrate_axes = [TestAxis(axis.lower())]

        max_smoothing = gcmd.get_float(
                "MAX_SMOOTHING", self.max_smoothing, minval=0.05)

        name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        if not self.is_valid_name_suffix(name_suffix):
            raise gcmd.error("Invalid NAME parameter")

        # Setup shaper calibration
        helper = shaper_calibrate.ShaperCalibrate(self.printer)

        calibration_data = self._run_test(gcmd, calibrate_axes, helper)

        configfile = self.printer.lookup_object('configfile')
        for axis in calibrate_axes:
            axis_name = axis.get_name()
            gcmd.respond_info(
                    "Calculating the best input shaper parameters for %s axis"
                    % (axis_name,))
            calibration_data[axis].normalize_to_frequencies()
            best_shaper, all_shapers = helper.find_best_shaper(
                    calibration_data[axis], max_smoothing, gcmd.respond_info)
            gcmd.respond_info(
                    "Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz"
                    % (axis_name, best_shaper.name,
                       axis_name, best_shaper.freq))
            helper.save_params(configfile, axis_name,
                               best_shaper.name, best_shaper.freq)
            csv_name = self.save_calibration_data(
                    'calibration_data', name_suffix, helper, axis,
                    calibration_data[axis], all_shapers)
            gcmd.respond_info(
                    "Shaper calibration data written to %s file" % (csv_name,))
            
        gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer.")
    cmd_MEASURE_AXES_NOISE_help = (
        "Measures noise of all enabled accelerometer chips")
    def cmd_MEASURE_AXES_NOISE(self, gcmd):
        meas_time = gcmd.get_float("MEAS_TIME", 2.)
        raw_values = [(chip_axis, chip.start_internal_client())
                      for chip_axis, chip in self.accel_chips]
        self.printer.lookup_object('toolhead').dwell(meas_time)
        for chip_axis, aclient in raw_values:
            aclient.finish_measurements()
        helper = shaper_calibrate.ShaperCalibrate(self.printer)
        for chip_axis, aclient in raw_values:
            data = helper.process_accelerometer_data(aclient)
            vx = data.psd_x.mean()
            vy = data.psd_y.mean()
            vz = data.psd_z.mean()
            gcmd.respond_info("Axes noise for %s-axis accelerometer: "
                              "%.6f (x), %.6f (y), %.6f (z)" % (
                                  chip_axis, vx, vy, vz))

    def is_valid_name_suffix(self, name_suffix):
        return name_suffix.replace('-', '').replace('_', '').isalnum()

    def get_filename(self, base, name_suffix, axis=None, point=None):
        name = base
        if axis:
            name += '_' + axis.get_name()
        if point:
            name += "_%.3f_%.3f_%.3f" % (point[0], point[1], point[2])
        name += '_' + name_suffix
        if self.target_dir == None:
            folder_name = "%s/%d/%d"%(self.target_folder_name,self.cur_pos,self.test.get_cur_height())
        else:
            folder_name = "%s/%d"%(self.target_dir,self.test.get_cur_height())
        if not os.path.exists(folder_name):
                os.makedirs(folder_name)
        return os.path.join(folder_name, name + ".csv")

    def save_calibration_data(self, base_name, name_suffix, shaper_calibrate,
                              axis, calibration_data, all_shapers=None):
        output = self.get_filename(base_name, name_suffix, axis)
        shaper_calibrate.save_calibration_data(output, calibration_data,
                                               all_shapers)
        return output

def load_config(config):
    return ResonanceTester(config)
