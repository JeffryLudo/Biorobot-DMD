
from pin_definitions import Pins
from machine import Pin, ADC


class EMG(object):

    def __init__(self, emg=1):
        """
        =INPUT=
            emg - int
                Choose emg between 1 to 3
        """
        if emg == 1:
            self.pin = Pin(Pins.EMG1, Pin.IN)
            self.adc = ADC(self.pin)
        elif emg == 2:
            self.pin = Pin(Pins.EMG2, Pin.IN)
            self.adc = ADC(self.pin)
        else:
            self.pin = Pin(Pins.EMG3, Pin.IN)
            self.adc = ADC(self.pin)

        return


    def read(self):
        return self.adc.read_u16()