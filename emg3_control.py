
from machine import Pin
from states import States
from potmeter import Potmeter
from emg import EMG
from emg_processing import EmgProcessing

class EMG3Control(object):

    def __init__(self, state_object, threshold):
        """
        =INPUT=
            state_object - boolean
                The state object
            threshold - int
                The threshold that the emg has to overpass in order to activate the change in state
        """
        self.state_object = state_object
        self.was_activated = False
        self.emg3 = EmgProcessing(emg=3) 
        self.is_abovethreshold = False
        self.is_belowthreshold = False
        self.threshold = threshold
        return

    
    def check_activation(self):
        """
        If EMG 3 goes above the threshold and then goes back below the threshold, it is seem as activated
        """
        # Read values from the potentiometer 
        self.value = self.emg3.process_signal()

        # Set a transition threshold for the emg
        if self.value > self.threshold:
            self.is_abovethreshold = True
        elif self.is_abovethreshold and self.value < self.threshold:
            self.is_belowthreshold = True
        
        # Check if the emg went above and below the threshold and activate the state
        if self.is_abovethreshold and self.is_belowthreshold:
            self.was_activated = True
            self.is_abovethreshold = False
            self.is_belowthreshold = False
        return

    
    def update_state(self):
        """
        Update state when EMG3 gets activeted
        """
        if self.was_activated:
            print('emg3 was activated')
            self.was_activated = False
            
            # Change states between XDIRECTION -> YDIRECTION -> XDIRECTION -> ...
            if self.state_object.state is States.XDIRECTION:
                self.state_object.set_state(States.YDIRECTION)
            elif self.state_object.state is States.YDIRECTION:
                self.state_object.set_state(States.XDIRECTION)
        return