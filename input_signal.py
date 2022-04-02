from ulab import vector as vc


class InputSignal(object):

    def __init__(self, T, amp):
        """
        =INPUT=
            T - int
                The period of the signal
            amp - int
                The amplitude of the signal
            Initialize the frequency and amplitude for a sinusoidal signal which is used for the identification response of the plant
        """
        self.pi = 3.1415926535897
        self.amp = amp
        freq = 1/T * 2*self.pi
        self.freq = freq
        return

    
    def sine_wave(self, t):
        """
        =INPUT=
            t - float
                The time steps for the sinusoidal signal
        """
        # Create a sin wave 
        y = self.amp*vc.sin(self.freq*t)
        return y



