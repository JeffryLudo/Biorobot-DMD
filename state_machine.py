from states import States
from state_object import StateObject
from state_functions import StateFunctions

from nucleo_button_control import NucleoButtonControl
from potmeter import Potmeter
from emg3_control import EMG3Control

# Micropython specific modules
import br_timer


class Robot(object):

    def __init__(self, timer_number, main_frequency):
        """
        =INPUT=
            timer_number - integer
                Timer number on the NUCLEO board
            main_frequency - integer
                Frequency at which the main ticker should run
        """
        # Initialize the ticker
        self.main_frequency = main_frequency
        self.main_ticker = br_timer.ticker(
            timer_number, main_frequency, self.run, True)

        # State object instance that can be updated by other objects
        self.state_object = StateObject()

        # Objects that can update the state object
        self.nucleo_button_control = NucleoButtonControl(self.state_object)
        self.state_functions = StateFunctions(self.state_object, main_frequency)
        self.emg3_control = EMG3Control(self.state_object, 12000)   # Threshold of 12000
        
        # Initialize motor off
        self.state_functions.motors_off()

        # The state machine itself
        self.state_machine = {
            States.READ: self.state_functions.read,
            States.HOME: self.state_functions.home,
            States.XDIRECTION: self.state_functions.x_motion,
            States.YDIRECTION: self.state_functions.y_motion
        }

        return


    def run(self):
        """
        Target for the ticker. Get's executed every time step.
        """
        if self.state_object.state is States.READ:
            # Check if the button was invoked for a state update
            self.nucleo_button_control.update_state()
        elif self.state_object.state is States.XDIRECTION or States.YDIRECTION:
            # Check if the button was invoked for a state update
            self.nucleo_button_control.update_state()
            # Check if EMG 3 was activated
            self.emg3_control.check_activation()
            self.emg3_control.update_state()


        # Run the active state from the state machine
        self.state_machine[self.state_object.state]()
        return


    def start(self):
        # Ticker start
        self.main_ticker.start()
        return


    def stop(self):
        # Ticker stop
        self.main_ticker.stop()
        return
