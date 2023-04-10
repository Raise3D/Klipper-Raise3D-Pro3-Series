# Printer cooling fan
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import pulse_counter

FAN_MIN_TIME = 0.100
IGNORE_FAN_NUM = 2
class Fan:
    def __init__(self, config, default_shutdown_speed=0.):
        self.printer = config.get_printer()
        self.last_fan_value = 0.
        self.last_fan_time = 0.
        # Read config
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1,
                                               minval=0.)
        self.off_below = config.getfloat('off_below', default=0.,
                                         minval=0., maxval=1.)
        cycle_time = config.getfloat('cycle_time', 0.010, above=0.)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        shutdown_speed = config.getfloat(
            'shutdown_speed', default_shutdown_speed, minval=0., maxval=1.)
        # Setup pwm object
        shutdown_power = max(0., min(self.max_power, shutdown_speed))
        self.mcu_fan = []
        self.fan_value = []
        self.max_fan_num = 0
        ppins = self.printer.lookup_object('pins')
        for i in range(10):
            sec = 'pin'
            if i:
                sec = 'pin%d'%(i,)
            if config.fileconfig.has_option(config.section,sec):
                mfan = ppins.setup_pin('pwm', config.get(sec))
                mfan.setup_max_duration(0.)
                mfan.setup_cycle_time(cycle_time, hardware_pwm) 
                mfan.setup_start_value(0., shutdown_power)
                self.mcu_fan.append(mfan)
                self.max_fan_num += 1
                self.fan_value.append(shutdown_power)
        # only for back fan test mode
        self.fan_value.append(shutdown_power)
        # Setup tachometer
        self.tachometer = FanTachometer(config)

        # Register callbacks
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
    def debug_str(self,msg):
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_raw(msg)
        pass
    def get_fan_value(self):
        out = ""
        fan_ptr = 0
        for val in self.fan_value:
            out = "%sS%d:%.1f "%(out,fan_ptr,val * 255)
            fan_ptr = fan_ptr+1
        return out
    def get_mcu(self):
        return self.mcu_fan.get_mcu()
    def set_speed(self, print_time, value , fan_num = 0):
        set_all_fan = False
        if value < self.off_below:
            value = 0.
        value = max(0., min(self.max_power, value * self.max_power))
        if fan_num < self.max_fan_num:
            mfan = self.mcu_fan[fan_num]
        else:
            set_all_fan = True
        if set_all_fan == False and self.fan_value[fan_num] == value:
            return
        if fan_num == 2:
            if value > 0:
                mcu = self.printer.lookup_object('mcu')
                if mcu != None:
                    mcu.ghead.set_normal_ghead_ctrl(ord('F'),fan_num,1)
                self.fan_value[fan_num] = 100
            elif value == 0:
                mcu = self.printer.lookup_object('mcu')
                if mcu != None:
                    mcu.ghead.set_normal_ghead_ctrl(ord('F'),fan_num,0)
                self.fan_value[fan_num] = 0
            return 
        if fan_num == IGNORE_FAN_NUM:
            return
        print_time = max(self.last_fan_time + FAN_MIN_TIME, print_time)
        if (value and value < self.max_power and self.kick_start_time
            and (not self.last_fan_value or value - self.last_fan_value > .5)):
            # Run fan at full speed for specified kick_start_time
            if set_all_fan == False:
                mfan.set_pwm(print_time, self.max_power)
            else:
                for mf in self.mcu_fan:
                    mf.set_pwm(print_time, self.max_power)    
            print_time += self.kick_start_time
        if set_all_fan == False:
            mfan.set_pwm(print_time, value)
            self.fan_value[fan_num] = value
            self.last_fan_time = print_time
        else:
            i = 0
            for mf in self.mcu_fan:
                mf.set_pwm(print_time, value) 
                self.fan_value[i] = value
                i += 1
            self.last_fan_time = print_time     
        self.last_fan_value = value
    def set_speed_from_command(self, value , fan_num = 0):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt:
                                              self.set_speed(pt, value, fan_num)))
    def _handle_request_restart(self, print_time):
        self.set_speed(print_time, 0.)

    def get_status(self, eventtime):
        tachometer_status = self.tachometer.get_status(eventtime)
        return {
            'speed': self.last_fan_value,
            'rpm': tachometer_status['rpm'],
        }

class FanTachometer:
    def __init__(self, config):
        printer = config.get_printer()
        self._freq_counter = None

        pin = config.get('tachometer_pin', None)
        if pin is not None:
            self.ppr = config.getint('tachometer_ppr', 2, minval=1)
            poll_time = config.getfloat('tachometer_poll_interval',
                                        0.0015, above=0.)
            sample_time = 1.
            self._freq_counter = pulse_counter.FrequencyCounter(
                printer, pin, sample_time, poll_time)

    def get_status(self, eventtime):
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30. / self.ppr
        else:
            rpm = None
        return {'rpm': rpm}

class PrinterFan:
    def __init__(self, config):
        self.fan = Fan(config)
        # Register commands
        self.get_fan_value = self.fan.get_fan_value
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("M106", self.cmd_M106)
        gcode.register_command("M107", self.cmd_M107)
    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)
    def cmd_M106(self, gcmd):
        # Set fan speed
        value = gcmd.get_float('S', 255., minval=0.) / 255.
        fan_num = gcmd.get_int('P', 255, minval = 0)
        self.fan.set_speed_from_command(value,fan_num)
    def cmd_M107(self, gcmd):
        # Turn fan off
        self.fan.set_speed_from_command(0.)

def load_config(config):
    return PrinterFan(config)
