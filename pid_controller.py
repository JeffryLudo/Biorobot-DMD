

class PID_pf(object):
    """
    PID in (pure) parallel form
    """

    def __init__(self, t_step, p_gain, i_gain, d_gain):
        """
        =INPUT=
            t_step - float
                Controller time step
            p_gain, i_gain, d_gain - float
                Proportional, integrative, differential controller gains
        """
        self.t_step = t_step
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

        self.past_error = 0
        self.integrated_error = 0

        return


    def step(self, reference, measured, filtfun=None):
        """
        =INPUT=
            reference - float
            measured - float
            filtfun - callable
                Callable which takes a sample as input and filters it.
                Used only on the differential signal.
        """

        # Compute error, integrated error, and differential error
        error = reference - measured
        self.integrated_error += error * self.t_step
        differential_error = (error - self.past_error) / self.t_step

        # Filter if a filter function handle was supplied
        if filtfun is not None:
            differential_error = filtfun(differential_error)
        
        self.past_error = error

        return (self.p_gain * error 
            + self.i_gain * self.integrated_error
            + self.d_gain * differential_error)

