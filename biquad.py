
class Biquad(object):

    def __init__(self, a_param, b_param):
        """
            a_param - tuple
                Holding the 3 coefficients (a0, a1, a2) for the denominator.
                All a's and b's are scaled such that a0 = 1.
            b_param - tuple
                Holding the 3 coefficients (b0, b1, b2) for the numerator
        """
        if len(a_param) != 3:
            raise ValueError('Incorrect number of coefficients for a')
        if len(b_param) != 3:
            raise ValueError('Incorrect number of coefficients for b')
        if a_param[0] != 1:
            a0, a1, a2 = a_param
            b0, b1, b2 = b_param
            a_param = (1, a1 / a0, a2 / a0)
            b_param = (b0 / a1, b1 / a1, b2 / a1)

        self.a_param = a_param
        self.b_param = b_param
        self.delay_register = [0.0, 0.0]
        return


    def step(self, sample):
        """
        Using transposed direct form 2
        """

        filtered_sample = sample * self.b_param[0] + self.delay_register[1]
        
        self.delay_register[1] = (self.delay_register[0] 
            + self.b_param[1] * sample
            - self.a_param[1] * filtered_sample)

        self.delay_register[0] = (self.b_param[2] * sample
                    - self.a_param[2] * filtered_sample)

        return filtered_sample
        