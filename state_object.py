from states import States


class StateObject(object):

    def __init__(self):
        self.last_state = None
        self.state = States.READ
        return


    def set_state(self, state):
        # Also updates the last_state
        self.last_state = self.state
        self.state = state
        return


    def is_new_state(self):
        # Also updates the last_state
        is_new = self.last_state is not self.state
        if is_new:
            self.last_state = self.state
        return is_new
