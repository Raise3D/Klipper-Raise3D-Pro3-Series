class PrinterGhead:
    def __init__(self,config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.min_temp = -273.0
        self.max_temp = 9999.9
        pheaters = self.printer.load_object(config,'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp,self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config,self)
        self.last_temp = 0.
        self.measured_min = 9999999.
        self.measured_max = 0.
        self.readtime = 0
        self.last_readtime = 0
    def temperature_callback(self,readtime,temp):
        self.last_temp = temp
        if temp:
            self.measured_min = min(self.measured_min,temp)
            self.measured_max = max(self.measured_max,temp)
        self.last_readtime = self.readtime
        self.readtime = readtime
    def get_temp(self,eventtime):
        return self.last_temp,0.

    def stats(self,eventtime):
        return False,'%s : temp = %.1f'%(self.name,self.last_temp)
    def get_status(self,eventtime):
        return {
            'temperature':self.last_temp,
            'measured_min_temp':self.measured_min,
            'measured_max_temp':self.measured_max,
        }
    def load_config_prefix(config):
        return PrinterGhead(config)
        # pheaters = config.get_printer().load_object(config, "heaters")
        # pheaters.add_sensor_factory("temperature_mcu", PrinterTemperatureMCU)