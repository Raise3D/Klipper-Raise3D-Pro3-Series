# Temperature measurements with thermistors
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import adc_temperature

KELVIN_TO_CELSIUS = -273.15

# Analog voltage to temperature converter for thermistors
MAX_ADC_VAL=3.272
MIN_ADC_VAL=1.378
NTC_3950_TABLE = ((-10,3.272),(-5,3.2642),(0,3.2526),(5,3.24),(10,3.224),(15,3.204436),
                    (20,3.18),(25,3.1518),(30,3.117894),(35,3.078),(40,3.031),(45,2.978416),
                    (50,2.918),(55,2.85),(60,2.7735),(65,2.69),(70,2.6),(75,2.50),(80,2.4),
                    (85,2.29),(90,2.177),(95,2.06),(100,1.94),(105,1.83),(110,1.71),(115,1.5975),(120,1.486),(125,1.378))
class Thermistor:
    def __init__(self, pullup, inline_resistor,is_lookup_table = False):
        self.pullup = pullup
        self.inline_resistor = inline_resistor
        self.c1 = self.c2 = self.c3 = 0.
        self.is_lookup_table = True
    def setup_coefficients(self, t1, r1, t2, r2, t3, r3, name=""):
        # Calculate Steinhart-Hart coefficents from temp measurements.
        # Arrange samples as 3 linear equations and solve for c1, c2, and c3.
        inv_t1 = 1. / (t1 - KELVIN_TO_CELSIUS)
        inv_t2 = 1. / (t2 - KELVIN_TO_CELSIUS)
        inv_t3 = 1. / (t3 - KELVIN_TO_CELSIUS)
        ln_r1 = math.log(r1)
        ln_r2 = math.log(r2)
        ln_r3 = math.log(r3)
        ln3_r1, ln3_r2, ln3_r3 = ln_r1**3, ln_r2**3, ln_r3**3

        inv_t12, inv_t13 = inv_t1 - inv_t2, inv_t1 - inv_t3
        ln_r12, ln_r13 = ln_r1 - ln_r2, ln_r1 - ln_r3
        ln3_r12, ln3_r13 = ln3_r1 - ln3_r2, ln3_r1 - ln3_r3

        self.c3 = ((inv_t12 - inv_t13 * ln_r12 / ln_r13)
                   / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13))
        if self.c3 <= 0.:
            beta = ln_r13 / inv_t13
            logging.warn("Using thermistor beta %.3f in heater %s", beta, name)
            self.setup_coefficients_beta(t1, r1, beta)
            return
        self.c2 = (inv_t12 - self.c3 * ln3_r12) / ln_r12
        self.c1 = inv_t1 - self.c2 * ln_r1 - self.c3 * ln3_r1
    def setup_coefficients_beta(self, t1, r1, beta):
        # Calculate equivalent Steinhart-Hart coefficents from beta
        inv_t1 = 1. / (t1 - KELVIN_TO_CELSIUS)
        ln_r1 = math.log(r1)
        self.c3 = 0.
        self.c2 = 1. / beta
        self.c1 = inv_t1 - self.c2 * ln_r1
    def lookup_table(self,val):
        for i in range(28):
            if val == NTC_3950_TABLE[i][1]:
                return NTC_3950_TABLE[i][0]
            elif val < NTC_3950_TABLE[i][1] and val > NTC_3950_TABLE[i + 1][1]:
                va = NTC_3950_TABLE[i][1]
                ta = NTC_3950_TABLE[i][0]
                vb = NTC_3950_TABLE[i + 1][1]
                tb = NTC_3950_TABLE[i + 1][0]
                temp = (tb - ta)*(va - val)/(va - vb) + ta
                return temp
    def calc_temp(self, adc):
        # Calculate temperature from adc
        adc = max(.00001, min(.99999, adc))
        if self.is_lookup_table == True:
            vol = adc * 3.3
            if vol > MAX_ADC_VAL:
                return 2000
            elif vol < MIN_ADC_VAL:
                return -2000
            else:
                temp = self.lookup_table(vol)
                return temp        
        else:
            r = self.pullup * adc / (1.0 - adc)
            ln_r = math.log(r - self.inline_resistor)
            inv_t = self.c1 + self.c2 * ln_r + self.c3 * ln_r**3
            temp2 =  1.0/inv_t + KELVIN_TO_CELSIUS
            return temp2
    def calc_adc(self, temp):
        # Calculate adc reading from a temperature
        if temp <= KELVIN_TO_CELSIUS:
            return 1.
        inv_t = 1. / (temp - KELVIN_TO_CELSIUS)
        if self.c3:
            # Solve for ln_r using Cardano's formula
            y = (self.c1 - inv_t) / (2. * self.c3)
            x = math.sqrt((self.c2 / (3. * self.c3))**3 + y**2)
            ln_r = math.pow(x - y, 1./3.) - math.pow(x + y, 1./3.)
        else:
            ln_r = (inv_t - self.c1) / self.c2
        r = math.exp(ln_r) + self.inline_resistor
        return r / (self.pullup + r)

# Create an ADC converter with a thermistor
def PrinterThermistor(config, params):
    pullup = config.getfloat('pullup_resistor', 4700., above=0.)
    inline_resistor = config.getfloat('inline_resistor', 0., minval=0.)
    thermistor = Thermistor(pullup, inline_resistor)
    if 'beta' in params:
        thermistor.setup_coefficients_beta(
            params['t1'], params['r1'], params['beta'])
    else:
        thermistor.setup_coefficients(
            params['t1'], params['r1'], params['t2'], params['r2'],
            params['t3'], params['r3'], name=config.get_name())
    return adc_temperature.PrinterADCtoTemperature(config, thermistor)

# Custom defined thermistors from the config file
class CustomThermistor:
    def __init__(self, config):
        self.name = " ".join(config.get_name().split()[1:])
        t1 = config.getfloat("temperature1", minval=KELVIN_TO_CELSIUS)
        r1 = config.getfloat("resistance1", minval=0.)
        beta = config.getfloat("beta", None, above=0.)
        if beta is not None:
            self.params = {'t1': t1, 'r1': r1, 'beta': beta}
            return
        t2 = config.getfloat("temperature2", minval=KELVIN_TO_CELSIUS)
        r2 = config.getfloat("resistance2", minval=0.)
        t3 = config.getfloat("temperature3", minval=KELVIN_TO_CELSIUS)
        r3 = config.getfloat("resistance3", minval=0.)
        (t1, r1), (t2, r2), (t3, r3) = sorted([(t1, r1), (t2, r2), (t3, r3)])
        self.params = {'t1': t1, 'r1': r1, 't2': t2, 'r2': r2,
                       't3': t3, 'r3': r3}
    def create(self, config):
        return PrinterThermistor(config, self.params)

# Default sensors
Sensors = {
    "EPCOS 100K B57560G104F": {
        't1': 25., 'r1': 100000., 't2': 150., 'r2': 1641.9,
        't3': 250., 'r3': 226.15 },
    "ATC Semitec 104GT-2": {
        't1': 20., 'r1': 126800., 't2': 150., 'r2': 1360.,
        't3': 300., 'r3': 80.65 },
    "SliceEngineering 450": {
        't1': 25., 'r1': 500000., 't2': 200., 'r2': 3734.,
        't3': 400., 'r3': 240. },
    "TDK NTCG104LH104JT1": {
        't1': 25., 'r1': 100000., 't2': 50., 'r2': 31230.,
        't3': 125., 'r3': 2066. },
    "NTC 100K beta 3950": { 't1': 25., 'r1': 100000., 'beta': 3950. },
    "Honeywell 100K 135-104LAG-J01": { 't1': 25., 'r1': 100000., 'beta': 3974.},
    "NTC 100K MGB18-104F39050L32": { 't1': 25., 'r1': 100000., 'beta': 4100. },
}

def load_config(config):
    # Register default thermistor types
    pheaters = config.get_printer().load_object(config, "heaters")
    for sensor_type, params in Sensors.items():
        func = (lambda config, params=params: PrinterThermistor(config, params))
        pheaters.add_sensor_factory(sensor_type, func)

def load_config_prefix(config):
    thermistor = CustomThermistor(config)
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(thermistor.name, thermistor.create)
