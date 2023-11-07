#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import datetime
import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

class INA219Driver():
    def __init__(self) -> None:
        i2c_bus = board.I2C()  # uses board.SCL and board.SDA
        # i2c_bus = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

        self.ina219 = INA219(i2c_bus)

        # display some of the advanced field (just to test)
        print("Config register:")
        print("  bus_voltage_range:    0x%1X" % self.ina219.bus_voltage_range)
        print("  gain:                 0x%1X" % self.ina219.gain)
        print("  bus_adc_resolution:   0x%1X" % self.ina219.bus_adc_resolution)
        print("  shunt_adc_resolution: 0x%1X" % self.ina219.shunt_adc_resolution)
        print("  mode:                 0x%1X" % self.ina219.mode)
        print("")

        # optional : change configuration to use 32 samples averaging for both bus voltage and shunt voltage
        self.ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        # optional : change voltage range to 16V
        self.ina219.bus_voltage_range = BusVoltageRange.RANGE_16V
        return
    
    def get_voltage(self) -> float:
        bus_voltage = self.ina219.bus_voltage  # voltage on V- (load side)
        shunt_voltage = self.ina219.shunt_voltage  # voltage between V+ and V- across the shunt
        volt = bus_voltage + shunt_voltage
        return volt
