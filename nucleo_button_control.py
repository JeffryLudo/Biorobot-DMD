
from machine import Pin
from pin_definitions import Pins
from states import States


class NucleoButtonControl(object):

    def __init__(self, state_object):
        self.state_object = state_object
        self.is_pressed = False
        self.button = Pin(Pins.NUCLEO_BLUE_BUTTON, Pin.IN)
        self.button.irq(self.callback, Pin.IRQ_FALLING)
        return


    def callback(self, pin):
        """
        Interrupt callback for button.
        =INPUT=
            pin - machine.Pin object. Unused.
        """
        self.is_pressed = True
        print('Button Press')
        return


    def update_state(self):
        """
        Update state on button press
        """
        
        if self.is_pressed:
            self.is_pressed = False
            
            # Change states between READ -> HOME -> XDIRECTION -> READ -> ...
            if self.state_object.state is States.READ:
                self.state_object.set_state(States.HOME)
            elif self.state_object.state is States.HOME:
                self.state_object.set_state(States.XDIRECTION)
            else:
                self.state_object.set_state(States.READ)


        return


