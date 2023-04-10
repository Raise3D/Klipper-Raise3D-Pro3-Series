import logging
class DoorHelper:
    def __init__(self, config):
        self.name = config.get_name().split()[-1] + "door"
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.is_door_opened = False
        self.sensor_enabled = True
        # Register commands and event handlers
        self.gcode.register_mux_command(
            "QUERY_DOOR_SENSOR", "SENSOR", self.name,
            self.cmd_QUERY_DOOR_SENSOR,
            desc=self.cmd_QUERY_DOOR_SENSOR_help)
        self.gcode.register_mux_command(
            "SET_DOOR_SENSOR", "SENSOR", self.name,
            self.cmd_SET_DOOR_SENSOR,
            desc=self.cmd_SET_DOOR_SENSOR_help)
    def note_filament_present(self, is_door_opened):
        if is_door_opened == self.is_door_opened:
            return
        #run anti-shake xxx
        self.is_door_opened = is_door_opened
        self.report_door_status()
    def report_door_status(self):
        if self.is_door_opened:
            msg = "%s : opened" % (self.name)
            event = "opened"
        else:
            msg = "%s : closed" % (self.name)
            event = "closed"
        self.gcode.respond_info(msg)
        self.printer.send_event("door:changed", self.name, event ,(1 - self.is_door_opened))
    def get_status(self, eventtime):
        return {
            "door_detected": bool(self.is_door_opened),
            "enabled": bool(self.sensor_enabled)}
    cmd_QUERY_DOOR_SENSOR_help = "Query the status of the Filament Sensor"
    def cmd_QUERY_DOOR_SENSOR(self, gcmd):
        if self.is_door_opened:
            msg = "%s : door opened" % (self.name)
        else:
            msg = "%s : door closed" % (self.name)
        gcmd.respond_info(msg)
    cmd_SET_DOOR_SENSOR_help = "Sets the filament sensor on/off"
    def cmd_SET_DOOR_SENSOR(self, gcmd):
        self.sensor_enabled = gcmd.get_int("ENABLE", 1)

class SwitchSensor:
    def __init__(self, config):
        printer = config.get_printer()
        buttons = printer.load_object(config, 'buttons')
        switch_pin = config.get('switch_pin')
        buttons.register_buttons([switch_pin], self._button_handler)
        self.door_helper = DoorHelper(config)
        self.get_status = self.door_helper.get_status
    def _button_handler(self, eventtime, state):
        self.door_helper.note_filament_present(state)

def load_config_prefix(config):
    return SwitchSensor(config)