from pin_definitions import Pins
from machine import Pin, ADC


class Potmeter(object):

    def __init__(self, meter=1):
        """
        =INPUT=
            meter - int
                Choose potentiometer between 1 and 2
        """
        if meter == 1:
            self.pin = Pin(Pins.POTMETER1, Pin.IN)
            self.adc = ADC(self.pin)
        else:
            self.pin = Pin(Pins.POTMETER2, Pin.IN)
            self.adc = ADC(self.pin)

        return


    def read(self):
        return self.adc.read_u16()
        
