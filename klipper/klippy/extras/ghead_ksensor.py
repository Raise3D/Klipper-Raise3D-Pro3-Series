
GHEAD_REPORT_TIME = 1
GHEAD_MIN_REPORT_TIME = 0.48

class GHEAD_KSENSOR:
    def __init__(self,config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.sensor_id = config.get("ghead_ptr")
        self.temp = self.min_temp = self.max_temp = 0.0
        self._report_clock = 0
        self.report_time = config.getfloat(
            'ghead_report_time',
            GHEAD_MIN_REPORT_TIME,
            minval = GHEAD_MIN_REPORT_TIME
        )
        self.mcu = self.printer.lookup_object('mcu')
        self.mcu.ghead.register_temphandle(self.sensor_id,self.temp_callback)
        self._callback = None
    def temp_callback(self,temp,readtime):
        if self._callback != None:
            self._callback(readtime,temp)
    def setup_minmax(self,min_temp,max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def fault(self,msg):
        self.printer.invoke_async_shutdown(msg)
    def setup_callback(self,cb):
        self._callback = cb
    def get_status(self,eventtime):
        return {
            'temperature':self.temp
        }
    def get_report_time_delta(self):
        return self.report_time
def load_config(config):
    pheaters = config.get_printer().load_object(config,"heaters")
    pheaters.add_sensor_factory("ghead_ksensor",GHEAD_KSENSOR)
