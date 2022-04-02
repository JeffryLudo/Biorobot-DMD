
from pin_definitions import Pins
from machine import Pin
from pyb import Timer
from timer_definitions import Timers


class Motor(object):

    def __init__(self, freq, encoder_period=50400, motor=1):
        """
        =INPUT=
            freq - int
                The frequency of the motors
            encoder_period - int
                The number of counts for a complete rotationof the main shaft, due to gear reatio of 1:6 50400 counts make one rotation
            motor - int
                Choose motor between 1 to 3
        """
        if motor == 1:
            # Configure driver pins
            pin = Pin(Pins.MOTOR1_PWM, Pin.OUT)
            timer_pwm = Timer(Timers.MOTOR1_PWM, freq=freq)
            self.pwm_pin = timer_pwm.channel(
                Timers.MOTOR1_PWM_CHANNEL, Timer.PWM, pin=pin)
            self.direction_pin = Pin(Pins.MOTOR1_DIRECTION, Pin.OUT)

            # Configure encoder pins
            pin_a = Pin(Pins.MOTOR1_ENC_A)
            pin_b = Pin(Pins.MOTOR1_ENC_B)
            self.timer_enc = Timer(
                Timers.MOTOR1_ENC, prescaler=0, period=encoder_period)
            self.timer_enc.channel(
                Timers.MOTOR1_ENC_A_CHANNEL, Timer.ENC_AB, pin=pin_a)
            self.timer_enc.channel(
                Timers.MOTOR1_ENC_B_CHANNEL, Timer.ENC_AB, pin=pin_b)

        else:
            # Configure driver pins
            pin = Pin(Pins.MOTOR2_PWM, Pin.OUT)
            timer_pwm = Timer(Timers.MOTOR2_PWM, freq=freq)
            self.pwm_pin = timer_pwm.channel(
                Timers.MOTOR2_PWM_CHANNEL, Timer.PWM, pin=pin)
            self.direction_pin = Pin(Pins.MOTOR2_DIRECTION, Pin.OUT)

            # Configure encoder pins
            pin_a = Pin(Pins.MOTOR2_ENC_A)
            pin_b = Pin(Pins.MOTOR2_ENC_B)
            self.timer_enc = Timer(
                Timers.MOTOR2_ENC, prescaler=0, period=encoder_period)
            self.timer_enc.channel(
                Timers.MOTOR2_ENC_A_CHANNEL, Timer.ENC_AB, pin=pin_a)
            self.timer_enc.channel(
                Timers.MOTOR2_ENC_B_CHANNEL, Timer.ENC_AB, pin=pin_b)

        return


    def pulse_width_percent(self, percentage):
        # Set the PWM of the motor
        self.pwm_pin.pulse_width_percent(percentage)
        return


    def reverse(self):
        # Flip the value on the direction pin
        reverse_value = not self.direction_pin.value()
        self.direction_pin.value(reverse_value)
        return

    
    def read_encoder_count(self):
        return self.timer_enc.counter()
