
class Unwrapper(object):

    def __init__(self, max_count, min_count=0, initial_count=0):
        """
        =INPUT=
            max_count - int
                Largest possible count when wrapped
            min_count - int (0)
                Smallest possible count when wrapped
            initial_count - int (0)

        =NOTES=
            It must hold that: max_count - min_count >= 2
        """
        if max_count - min_count < 2:
            raise ValueError("Invalid value for max_count and/or min_count")
        if initial_count < min_count:
            print('Warning: initial_count < min_count. Setting it to min_count.')
            initial_count = min_count
        if initial_count > max_count:
            print('Warning: initial_count > max_count. Setting it to max_count.')
            initial_count = max_count

        self.max_count = max_count
        self.min_count = min_count
        self.period = max_count - min_count

        self.count_wrapped = initial_count
        self.count_unwrapped = initial_count
        self.wrap_count = 0
        return


    def unwrap(self, count):
        """
        =INPUT=
            count - int
                The count to unwrap
        """

        change = count - self.count_wrapped
        change_on_wrap = ((self.min_count - self.count_wrapped) +
            (self.max_count - count) + 1)
        self.count_wrapped = count

        # Update unwrapped count
        # Wrap decrement
        if change > self.period / 2:
            self.wrap_count -= 1
            self.count_unwrapped -= change_on_wrap

        # Wrap increment
        elif change < -self.period / 2:
            self.wrap_count += 1
            self.count_unwrapped += change_on_wrap
        
        # No wrap happened
        else:
            self.count_unwrapped += change

        return self.count_unwrapped


    def reset(self):
        self.count_unwrapped = 0
        return