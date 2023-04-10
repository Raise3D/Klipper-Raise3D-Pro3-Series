# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from distutils import cmd
import logging, threading


######################################################################
# Heater
######################################################################

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 10.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.
TEMP_ERROR_VAL = 15
MAX_TEMP_ERROR_COUNT = 10

class Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        # self.pwm_delay = 1.5
        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            'min_extrude_temp', 170.,
            minval=self.min_temp, maxval=self.max_temp)
        is_fileoutput = (self.printer.get_start_args().get('debugoutput')
                         is not None)
        # self.can_extrude = self.min_extrude_temp <= 0. or is_fileoutput
        self.can_extrude = False
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.recv_temp_count = 0.
        self.last_temp_time = 0.
        self.lost_print_time = 0.
        # pwm caching
        self.next_pwm_time = 0.
        self.last_pwm_value = 0.
        self.temp_error_count = 0
        self.last_error_temp = 0.
        self.heating_fault_count = 0
        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control_type = config.get('control')
        self.control = algo(self, config)
        # Setup output heater pin
        heater_pin = config.get('heater_pin')
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', heater_pin)
        pwm_cycle_time = config.getfloat('pwm_cycle_time', 0.100, above=0.,
                                         maxval=self.pwm_delay)
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (self.name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_HEATER_TEMPERATURE", "HEATER",
                                   self.name, self.cmd_SET_HEATER_TEMPERATURE,
                                   desc=self.cmd_SET_HEATER_TEMPERATURE_help)
        # respond oper
        self.respond_info = gcode.respond_info
        self.respond_raw = gcode.respond_raw
        self.respond_warning = gcode._respond_warning
        self.respond_error = gcode._respond_error
        self.extruder_num = 10
    def set_pid(self,gcmd):
        if self.control_type != 'pid':
            gcmd.respond_raw("warning:could not set pid val for bangbang!!")
        self.control.set_pid(self.name,self.printer,gcmd)
    def nprintf(self,msg):
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_raw(msg)
    def set_extruder_num(self,num):
        self.extruder_num = num
        pass
    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.:
            value = 0.
        #self.nprintf("read_time = %f , next_pwm_time = %f ,last_pwm_value = %f"%(read_time,self.next_pwm_time,self.last_pwm_value))
        if ((read_time < self.next_pwm_time or not self.last_pwm_value)
            and abs(value - self.last_pwm_value) < 0.05):
            #print("No significant change in value - can suppress update")
            #print("read_time = %f , next_pwm_time = %f ,last_pwm_value = %f"%(read_time,self.next_pwm_time,self.last_pwm_value))
            return
        pwm_time = read_time + self.pwm_delay
        #self.next_pwm_time = pwm_time + 0.5 * MAX_HEAT_TIME
        self.next_pwm_time = pwm_time 
        self.lost_print_time = pwm_time + 3.
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(pwm_time, value)
        #print ("pwm delay = %.3f"%self.pwm_delay)
        # if self.name == "extruder":
        #     self.nprintf("pwm val = %.2f"%value)
        # self.nprintf("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
        #               self.name, value, pwm_time,
        #               self.last_temp, self.last_temp_time, self.target_temp)
    def reset_temp(self,temp = 0):
        self.last_temp = self.smoothed_temp = self.target_temp = 0. 
        self.temp_error_count = 0
        self.recv_temp_count  = 0
    def reset_heater_stat(self,temp = 0):
        self.last_temp = self.smoothed_temp = self.target_temp = 0. 
        self.temp_error_count = 0
        self.recv_temp_count  = 0

    def temperature_callback(self, read_time, temp):
        #check 
        # self.nprintf("heater %d temp = %.2f"%(self.extruder_num,temp))
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            if self.smoothed_temp >= self.min_extrude_temp:
                self.heating_fault_count = 0
                self.can_extrude = True
            else:
                if self.can_extrude == True:
                    self.heating_fault_count = self.heating_fault_count + 1
                    if self.heating_fault_count >= 5:
                        self.heating_fault_count = 0
                        self.can_extrude = False
                        if self.extruder_num < 2:
                            pass
                            # self.respond_error("Heating fault on heater %d temperature excursion exceeded %.2f"%(self.extruder_num+1,self.smoothed_temp))
            # self.can_extrude = (self.smoothed_temp >= self.min_extrude_temp)
            if self.smoothed_temp < 40 and self.target_temp == 0 and self.extruder_num <= 1:
                self.printer.send_event("heat:stop",self.extruder_num)
    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            self.target_temp = degrees
        if self.target_temp > 0:
            self.printer.send_event("heat:start",self.extruder_num)
    def get_temp(self, eventtime):
        print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp)
    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.
        return old_control
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.name, target_temp, last_temp, last_pwm_value)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
        return {'temperature': smoothed_temp, 'target': target_temp,
                'power': last_pwm_value}
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp)


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_update(self, read_time, temp, target_temp):
        # print("bangbang curT = %f targetT %f"%(temp,target_temp))
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.)
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp-self.max_delta


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 2.
PID_SETTLE_SLOPE = 1.

class ControlPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        config.deprecate('pid_integral_max')
        imax = config.getfloat('pid_integral_max', self.heater_max_power,
                               minval=0.)
        self.temp_integ_max = 0.
        if self.Ki:
            self.temp_integ_max = imax / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
    def set_pid_val(self,printer,name,kp,ki,kd):
        section = name
        self.Kp = kp/PID_PARAM_BASE
        self.Ki = ki/PID_PARAM_BASE
        self.Kd = kd/PID_PARAM_BASE
        printer._save_config(section,'pid_Kp',kp)
        printer._save_config(section,'pid_Ki',ki)
        printer._save_config(section,'pid_Kd',kd)
        gcode = printer.lookup_object('gcode')
        gcode.respond_raw("ok %s to set pid"%name)
        pass
    def set_pid(self,name,printer,gcmd=None):
        cmdlist = gcmd._params
        section = name
        lent = int(len(cmdlist))
        if lent > 1:
            if 'P' in cmdlist or 'I' in cmdlist or 'D' in cmdlist:
                need_save = gcmd.get_int('SAVE', 1)
                self.Kp = gcmd.get_float('P', self.Kp*PID_PARAM_BASE) / PID_PARAM_BASE
                self.Ki = gcmd.get_float('I', self.Ki*PID_PARAM_BASE) / PID_PARAM_BASE
                self.Kd = gcmd.get_float('D', self.Kd*PID_PARAM_BASE) / PID_PARAM_BASE
                if section is not None and need_save > 0:
                    printer._save_config(section,'pid_Kp',self.Kp*PID_PARAM_BASE)
                    printer._save_config(section,'pid_Ki',self.Ki*PID_PARAM_BASE)
                    printer._save_config(section,'pid_Kd',self.Kd*PID_PARAM_BASE)
        #     if section is not None:
        #         gcmd.respond_raw("%s P:%.2f I:%.2f D:%.2f"%(section,self.Kp*PID_PARAM_BASE,self.Ki*PID_PARAM_BASE,self.Kd*PID_PARAM_BASE))
        # elif lent == 1:
        #     if section is not None:
        #         gcmd.respond_raw("%s P:%.2f I:%.2f D:%.2f"%(section,self.Kp*PID_PARAM_BASE,self.Ki*PID_PARAM_BASE,self.Kd*PID_PARAM_BASE))
        if name == 'extruder':
            gcmd.respond_raw("ok e:0 p:%.2f i:%.2f d:%.2f"%(self.Kp*PID_PARAM_BASE,self.Ki*PID_PARAM_BASE,self.Kd*PID_PARAM_BASE))
        elif name == 'extruder1':
            gcmd.respond_raw("ok e:1 p:%.2f i:%.2f d:%.2f"%(self.Kp*PID_PARAM_BASE,self.Ki*PID_PARAM_BASE,self.Kd*PID_PARAM_BASE))
        elif name == 'heater_bed':
            gcmd.respond_raw("ok b:0 p:%.2f i:%.2f d:%.2f"%(self.Kp*PID_PARAM_BASE,self.Ki*PID_PARAM_BASE,self.Kd*PID_PARAM_BASE))
    def temperature_update(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        #logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0., min(self.heater_max_power, co))

        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)


######################################################################
# Sensor and heater lookup
######################################################################

class PrinterHeaters:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.heaters = {}
        self.gcode_id_to_sensor = {}
        self.available_heaters = []
        self.available_sensors = []
        self.has_started = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("gcode:request_restart",
                                            self.turn_off_all_heaters)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc=self.cmd_TURN_OFF_HEATERS_help)
        gcode.register_command("M105", self.cmd_M105, when_not_ready=True)
        gcode.register_command("M303", self.cmd_M303)
        gcode.register_command("M307", self.cmd_M307)

        gcode.register_command("TEMPERATURE_WAIT", self.cmd_TEMPERATURE_WAIT,
                               desc=self.cmd_TEMPERATURE_WAIT_help)
    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory
    def setup_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Setup sensor
        sensor = self.setup_sensor(config)
        # Create heater

        self.heaters[heater_name] = heater = Heater(config, sensor)
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater
    def get_all_heaters(self):
        return self.available_heaters
    def lookup_heater(self, heater_name):
        if heater_name not in self.heaters:
            return None
            # raise self.printer.config_error(
            #     "Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]
    def setup_sensor(self, config):
        modules = ["thermistor", "adc_temperature", "spi_temperature",
                   "bme280", "htu21d", "lm75", "temperature_host",
                   "temperature_mcu", "ds18b20" ,"ghead_ksensor"]

        for module_name in modules:
            self.printer.load_object(config, module_name)
        sensor_type = config.get('sensor_type')
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,))
        return self.sensor_factories[sensor_type](config)
    def register_sensor(self, config, psensor, gcode_id=None):
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
    def get_status(self, eventtime):
        return {'available_heaters': self.available_heaters,
                'available_sensors': self.available_sensors}
    def turn_off_all_heaters(self, print_time=0.):
        for heater in self.heaters.values():
            heater.set_temp(0.)
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.turn_off_all_heaters()
    # G-Code M105 temperature reporting
    def _handle_ready(self):
        self.has_started = True
    def _get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_temp(eventtime)
                out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        if not out:
            return "T:0 "
        return " ".join(out) + " "
    def cmd_M307(self,gcmd):
        ptr = gcmd.get_int('H',None)
        val_f = gcmd.get_float('S',None)
        pheaters = self.printer.lookup_object('heaters')
        section = "default"
        if ptr != None and pheaters != None and val_f != None:
            if ptr == 0:
                # heater_bed
                section = "heater_bed"
                pass
            elif ptr == 1:
                # left heater
                section = "extruder"
            elif ptr == 2:
                # right heater
                section = "extruder1"
                pass
            heater = pheaters.lookup_heater(section)
            if heater != None and section != "default":
                control = heater.control
                if heater.max_power != val_f:
                    heater.max_power = val_f
                    control.heater_max_power = heater.max_power
                    self.printer._save_config(section,'max_power',heater.max_power)
                gcmd.respond_raw("ok %s max_power: %.1f "%(section,heater.max_power))
        else:
            gcmd.respond_raw("ivalid parameters....")
        pass
    def cmd_M303(self,gcmd):
        cmdlist = gcmd._params
        gcode = self.printer.lookup_object('gcode')
        if 'E' in cmdlist and 'S' in cmdlist:
            val = int(cmdlist['E'])
            if val > 1:
                gcmd._respond_warning("ivalid parameters")
                return
            tmp = float(cmdlist['S'])
            section = "extruder"
            count = gcmd.get_int('C',0)
            if val > 0:
               section = "extruder%d"%(val,)
            gcode.run_script_from_command("PID_CALIBRATE HEATER=%s TARGET=%.1f COUNT=%d"%(section,tmp,count))
        elif 'B' in cmdlist and 'S' in cmdlist:
            tmp = float(cmdlist['S'])
            count = gcmd.get_int('C',1)
            gcode.run_script_from_command("PID_CALIBRATE HEATER=heater_bed TARGET=%.1f COUNT=%d"%(tmp,count))
        pass
    def cmd_M105(self, gcmd):
        # Get Extruder Temperature
        toolhead = self.printer.check_object("toolhead")
        reactor = self.printer.get_reactor()
        if hasattr(toolhead,"get_extruder_temp_report"):
            curtemp = toolhead.get_extruder_temp_report()
        else:
            curtemp = ""
        fan = self.printer.check_object("fan")
        if fan != None:
            fanvalue = fan.get_fan_value()
        else:
            fanvalue = ""
        gcode_move = self.printer.lookup_object('gcode_move')
        if hasattr(gcode_move,"get_speed_factor"):
            printspeed = gcode_move.get_speed_factor()
        else:
            printspeed = ""
        if hasattr(gcode_move,"get_extruder_factor"):
            extruderspeed = gcode_move.get_extruder_factor()
        else:
            extruderspeed = ""
        msg =str(curtemp) + self._get_temp(reactor.monotonic()) + str(fanvalue) + str(printspeed) + str(extruderspeed)
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)
    def _wait_for_temperature(self, heater):
        # Helper to wait on heater.check_busy() and report M105 temperatures
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and heater.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)
    def set_temperature(self, heater, temp, wait=False):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        heater.set_temp(temp)
        if wait and temp:
            self._wait_for_temperature(heater)
    cmd_TEMPERATURE_WAIT_help = "Wait for a temperature on a sensor"
    def cmd_TEMPERATURE_WAIT(self, gcmd):
        sensor_name = gcmd.get('SENSOR')
        if sensor_name not in self.available_sensors:
            raise gcmd.error("Unknown sensor '%s'" % (sensor_name,))
        min_temp = gcmd.get_float('MINIMUM', float('-inf'))
        max_temp = gcmd.get_float('MAXIMUM', float('inf'), above=min_temp)
        if min_temp == float('-inf') and max_temp == float('inf'):
            raise gcmd.error(
                "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM.")
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        if sensor_name in self.heaters:
            sensor = self.heaters[sensor_name]
        else:
            sensor = self.printer.lookup_object(sensor_name)
        toolhead = self.printer.lookup_object("toolhead")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown():
            temp, target = sensor.get_temp(eventtime)
            if temp >= min_temp and temp <= max_temp:
                return
            print_time = toolhead.get_last_move_time()
            gcmd.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

def load_config(config):
    return PrinterHeaters(config)
