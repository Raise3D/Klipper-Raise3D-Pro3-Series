# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper

gl_init_extruder = True

class PrinterExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.ghead = self.printer.lookup_object('mcu').get_ghead()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        shared_heater = config.get('shared_heater', None)
        pheaters = self.printer.load_object(config, 'heaters')
        gcode_id = 'T%d' % (extruder_num,)
        self.extruder_num = extruder_num
        self.extruder_speed_factor = 100.
        # print("add extruder == %d"%extruder_num)
        if shared_heater is None:
            self.heater = pheaters.setup_heater(config, gcode_id)
        else:
            self.heater = pheaters.lookup_heater(shared_heater)
        self.heater.set_extruder_num(extruder_num)
        self.stepper = stepper.PrinterStepper(config)
        self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        filament_diameter = config.getfloat(
            'filament_diameter', minval=self.nozzle_diameter)
        self.filament_area = math.pi * (filament_diameter * .5)**2
        def_max_cross_section = 4. * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            'max_extrude_cross_section', def_max_cross_section, above=0.)
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
        self.pressure_advance = self.pressure_advance_smooth_time = 0.
        pressure_advance = config.getfloat('pressure_advance', 0., minval=0.)
        smooth_time = config.getfloat('pressure_advance_smooth_time',
                                      0.040, above=0., maxval=.200)
        
        # event
        self.printer.register_event_handler("ghead:hotend", self.hotend_handle)

        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.stepper.set_trapq(self.trapq)
        toolhead.register_step_generator(self.stepper.generate_steps)
        self.extruder_set_smooth_time = ffi_lib.extruder_set_smooth_time
        self._set_pressure_advance(pressure_advance, smooth_time)
        self.x_offset = 0.
        self.y_offset = 0.
        self.z_offset = 0.
        self.co_offset = {'X':self.x_offset,'Y':self.x_offset,'Z':self.x_offset}
        self.is_hotend_in = True
        self.record_last_heating_target = 0.
        self.is_anti_shaking = False
        # Register commands
        self.gcode = gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
            gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", None,
                                       self.cmd_default_SET_PRESSURE_ADVANCE,
                                       desc=self.cmd_SET_PRESSURE_ADVANCE_help)
            gcode.register_command("M301", self.cmd_M301)
            gcode.register_command("M572", self.cmd_M572)
        gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_PRESSURE_ADVANCE,
                                   desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
        gcode.register_mux_command("SET_EXTRUDER_STEP_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_E_STEP_DISTANCE,
                                   desc=self.cmd_SET_E_STEP_DISTANCE_help)
        self.hotend_moniter_timer = self.reactor.register_timer(
                self.lost_hotend_handle, self.reactor.NEVER)
    def lost_hotend_handle(self,eventtime):
        if self.extruder_num == 0:
            hotend_ptr = 'left'
        elif self.extruder_num == 1:
            hotend_ptr = 'right'
        self.is_hotend_in = False
        self.is_anti_shaking = False
        # stop heat working
        pheaters = self.printer.lookup_object('heaters')
        self.gcode._respond_warning("%s hotend is not in!"%hotend_ptr)
        # pheaters.set_temperature(self.heater, 0, False)
        self.heater.reset_heater_stat(0)
        if self.record_last_heating_target > 0:
            self.record_last_heating_target = 0
            # self.heater.set_pwm(self.heater.lost_print_time,0)
            # self.ghead.query_temp(self.extruder_num)
        self.nprintf("record_last_heating_target = %.1f"%self.record_last_heating_target)
        return self.reactor.NEVER
    def nprintf(self,str):
        self.gcode.respond_raw(str)
    def hotend_handle(self,hotend_ptr,val):
        pheaters = self.printer.lookup_object('heaters')
        if hotend_ptr == 'left':
            extruder_num = 0
        elif hotend_ptr == 'right':
            extruder_num = 1
        if extruder_num == self.extruder_num:
            if val == 0 :
                self.is_hotend_in = True
                self.is_anti_shaking = False
                self.reactor.update_timer(self.hotend_moniter_timer,self.reactor.NEVER)
                if self.record_last_heating_target > 0:
                    pheaters.set_temperature(self.heater, self.record_last_heating_target, False)
                    self.printer.send_event("heat:start",extruder_num)
                    self.nprintf("resume heating!")
                    self.record_last_heating_target = 0
            else:
                if self.is_hotend_in == True:
                    if self.is_anti_shaking == False:
                        self.is_anti_shaking = True
                        eventtime = self.reactor.monotonic()
                        # if it is heating now
                        if self.heater.target_temp > 0:
                            self.record_last_heating_target = self.heater.target_temp
                            pheaters.set_temperature(self.heater, 0, False)
                            # self.ghead.query_temp(self.extruder_num)
                            self.nprintf("stop heating temporary!")
                            self.nprintf("record_last_heating_target = %.1f"%self.record_last_heating_target)
                        else:
                            self.record_last_heating_target = 0
                        self.gcode.respond_raw("anti-shake start!")
                        self.reactor.update_timer(self.hotend_moniter_timer,eventtime + 3.)
                        pass
                    
    def get_extruder_offset(self):
        return (self.x_offset,self.y_offset,self.z_offset)
    def get_co_offset(self):
        return self.co_offset
    def set_extruder_offset(self,offset):
        if len(offset) == 3:
            if offset[0] != self.x_offset:
                self.x_offset = offset[0]
                self.co_offset['X'] = offset[0]
            if offset[1] != self.y_offset:
                self.y_offset = offset[1]
                self.co_offset['Y'] = offset[1]
            if offset[2] != self.z_offset:
                self.z_offset = offset[2]
                self.co_offset['Z'] = offset[2]
    def set_extruder_factor(self,factor):
        self.extruder_speed_factor = factor
    def get_extruder_factor(self):
        return self.extruder_speed_factor
    def set_e_jerk(self,jerk):
        self.instant_corner_v = jerk
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_raw("extruder N%d ejerk = %.2f"%(self.extruder_num,self.instant_corner_v))
    def get_e_jerk(self):
        return self.instant_corner_v
    def update_move_time(self, flush_time):
        self.trapq_finalize_moves(self.trapq, flush_time)
    def _set_pressure_advance(self, pressure_advance, smooth_time):
        old_smooth_time = self.pressure_advance_smooth_time
        if not self.pressure_advance:
            old_smooth_time = 0.
        new_smooth_time = smooth_time
        if not pressure_advance:
            new_smooth_time = 0.
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.note_step_generation_scan_time(new_smooth_time * .5,
                                                old_delay=old_smooth_time * .5)
        self.extruder_set_smooth_time(self.sk_extruder, new_smooth_time)
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time
    def get_status(self, eventtime):
        return dict(self.heater.get_status(eventtime),
                    can_extrude=self.heater.can_extrude,
                    pressure_advance=self.pressure_advance,
                    smooth_time=self.pressure_advance_smooth_time)
    def get_name(self):
        return self.name
    def get_heater(self):
        return self.heater
    def get_trapq(self):
        return self.trapq
    def sync_stepper(self, stepper):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        epos = self.stepper.get_commanded_position()
        stepper.set_position([epos, 0., 0.])
        stepper.set_trapq(self.trapq)
    def stats(self, eventtime):
        return self.heater.stats(eventtime)
    def check_move(self, move):
        axis_r = move.axes_r[3]
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[3]) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[3], self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[3] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
    def get_extruder_num(self):
        return self.extruder_num
    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3] - prev_move.axes_r[3]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def move(self, print_time, move):
        axis_r = move.axes_r[3]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        pressure_advance = 0.
        if axis_r > 0. and (move.axes_d[0] or move.axes_d[1]):
            pressure_advance = self.pressure_advance
        # Queue movement (x is extruder movement, y is pressure advance)
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          move.start_pos[3], 0., 0.,
                          1., pressure_advance, 0.,
                          start_v, cruise_v, accel)
    def find_past_position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return self.stepper.mcu_to_commanded_position(mcu_pos)
    def cmd_M301(self, gcmd):
        cmdlist = gcmd._params
        if 'H' in cmdlist:
            val = int(cmdlist['H'])
            section = "extruder"
            if val > 0:
                section = "extruder%d"%(val,) 
            extruder = self.printer.check_object(section)
            if extruder != None:
                extruder.heater.set_pid(gcmd)
    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float('S', 0.)
        index = gcmd.get_int('T', None, minval=0)
        if index is not None:
            section = 'extruder'
            if index:
                section = 'extruder%d' % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                if temp <= 0.:
                    return
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object('toolhead').get_extruder()
            # index = extruder.extruder_num
        if extruder is None:
            extruder = self.printer.lookup_object("extruder", None)
        if extruder != None and extruder.is_hotend_in == True:
            ptr = extruder.get_extruder_num()
            pheaters = self.printer.lookup_object('heaters')
            pheaters.set_temperature(extruder.get_heater(), temp, wait)
            if temp >= 40:
                self.nprintf("send heat start event")
                self.printer.send_event("heat:start",ptr)
            elif temp == 0:
                extruder.record_last_heating_target = 0
        elif extruder.is_hotend_in == False:
            self.nprintf("hotend %d is not in"%extruder.extruder_num)
            pass
        else:
            pass
    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)
    cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"
    def cmd_default_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        extruder.cmd_SET_PRESSURE_ADVANCE(gcmd)
    def cmd_M572(self, gcmd):
        cmdlist = gcmd._params
        need_save = gcmd.get_int('SAVE', 0)
        if 'D' in cmdlist:
            val = int(cmdlist['D']) 
            section = "extruder"
            if val > 0:
                section = "extruder%d"%(val,) 
            extruder = self.printer.check_object(section)
            if extruder != None:
                if 'S' in cmdlist:
                    val = float(cmdlist['S'])
                    pressure_advance = val
                    if 'T' in cmdlist:
                        smooth_time = float(cmdlist['T'])
                    else:
                        smooth_time = extruder.pressure_advance_smooth_time
                    extruder._set_pressure_advance(pressure_advance, smooth_time)
                    msg = ("ok pressure_advance: %.6f\n"
                        "pressure_advance_smooth_time: %.6f"
                        % (pressure_advance, smooth_time))
                    extruder.printer.set_rollover_info(extruder.name, "%s: %s" % (extruder.name, msg))
                    gcmd.respond_info(msg, log=False)
                    if need_save > 0:
                        section = 'extruder'
                        if extruder.extruder_num > 0 :
                            section = "extruder%d"%(extruder.extruder_num,)
                        extruder.printer._save_config(section,'pressure_advance',extruder.pressure_advance)
                else:
                    msg = ("ok pressure_advance: %.6f\n"
                        "pressure_advance_smooth_time: %.6f"
                        % (extruder.pressure_advance, extruder.pressure_advance_smooth_time))
                    gcmd.respond_info(msg, log=False)
    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        pressure_advance = gcmd.get_float('ADVANCE', self.pressure_advance,
                                          minval=0.)
        smooth_time = gcmd.get_float('SMOOTH_TIME',
                                     self.pressure_advance_smooth_time,
                                     minval=0., maxval=.200)
        self._set_pressure_advance(pressure_advance, smooth_time)
        msg = ("pressure_advance: %.6f\n"
               "pressure_advance_smooth_time: %.6f"
               % (pressure_advance, smooth_time))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcmd.respond_info(msg, log=False)
    cmd_SET_E_STEP_DISTANCE_help = "Set extruder step distance"
    def cmd_SET_E_STEP_DISTANCE(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        dist = gcmd.get_float('DISTANCE', None, above=0.)
        if dist is None:
            step_dist = self.stepper.get_step_dist()
            gcmd.respond_info("Extruder '%s' step distance is %0.6f"
                              % (self.name, step_dist))
            return
        toolhead.flush_step_generation()
        self.stepper.set_step_dist(dist)
        gcmd.respond_info("Extruder '%s' step distance set to %0.6f"
                          % (self.name, dist))
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        global gl_init_extruder
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            gl_init_extruder = False
            return
        if gl_init_extruder:
            gl_init_extruder = False
        else:
            self.printer.send_event("extruder:do_before_active")
        self.gcode.run_script_from_command("SET_GCODE_OFFSET X=%.2f Y=%.2f Z=%.2f MOVE=0"%(self.x_offset, self.y_offset, self.z_offset))
        self.ghead.switch_tool(self.extruder_num)
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.stepper.get_commanded_position())
        self.printer.send_event("extruder:activate_extruder")

# Dummy extruder class used when a printer has no extruder at all
class DummyExtruder:
    def __init__(self, printer):
        self.printer = printer
    def update_move_time(self, flush_time):
        pass
    def check_move(self, move):
        raise move.move_error("Extrude when no extruder present")
    def find_past_position(self, print_time):
        return 0.
    def calc_junction(self, prev_move, move):
        return move.max_cruise_v2
    def get_name(self):
        return ""
    def get_heater(self):
        raise self.printer.command_error("Extruder not configured")
    def get_trapq(self):
        raise self.printer.command_error("Extruder not configured")

def add_printer_objects(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'extruder'
        if i:
            section = 'extruder%d' % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
