# Declare all timers and channels to be used


class Timers(object):
    """
    The ticker that calls run() of Robot may NOT use the same timer.
    """
    RUN = 2
    MOTOR1_PWM = 1
    MOTOR1_PWM_CHANNEL = 2
    MOTOR1_ENC = 4
    MOTOR1_ENC_A_CHANNEL = 1
    MOTOR1_ENC_B_CHANNEL = 2

    MOTOR2_PWM = 1
    MOTOR2_PWM_CHANNEL = 1
    MOTOR2_ENC = 3
    MOTOR2_ENC_A_CHANNEL = 1
    MOTOR2_ENC_B_CHANNEL = 2
