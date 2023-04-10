class Led_light:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('M355',self.switch_led_light,desc = None)
        l_led_pin = config.get('left_light_pin')
        r_led_pin = config.get('right_light_pin')
        ppins = self.printer.lookup_object('pins')
        # if l_led_pin != None:
        #     self.left_light = ppins.setup_pin('digital_out', l_led_pin)
        # if r_led_pin != None:
        #     self.right_light = ppins.setup_pin('digital_out', r_led_pin)
    def switch_led_light(self,gcmd):
        cmdlist = gcmd._params
        if "S" in cmdlist:
            val = int(cmdlist["S"])
            if val == 0:
                mcu = self.printer.lookup_object('mcu')
                if mcu != None:
                    mcu.ghead.set_normal_ghead_ctrl(mode = ord('L'),hs = 1)
            else:
                mcu = self.printer.lookup_object('mcu')
                if mcu != None:
                    mcu.ghead.set_normal_ghead_ctrl(mode = ord('L'),hs = 0)
def load_config(config):
    return Led_light(config)
