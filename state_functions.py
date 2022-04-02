
import br_serial
from potmeter import Potmeter
from motor import Motor
from biquad import Biquad
from unwrapper import Unwrapper
from pid_controller import PID_pf
from states import States
from input_signal import InputSignal
from kinematics import Kinematics
from emg_processing import EmgProcessing
from machine import Pin
from pin_definitions import Pins



class StateFunctions(object):

    def __init__(self, state_object, main_frequency):
        
        # State object
        self.state_object = state_object

        # Allow to visualize the signals
        self.serial_pc = br_serial.serial_pc(3)
        
        # Potometers
        self.pot1 = Potmeter(meter=1)
        self.pot2 = Potmeter(meter=2)
        
        # Set EMG signals
        self.emg1 = EmgProcessing(emg=1)
        self.emg2 = EmgProcessing(emg=2)
        self.emg3 = EmgProcessing(emg=3) 

        # Set the motors
        self.motor2 = Motor(main_frequency, encoder_period=50400, motor=2)
        self.motor1 = Motor(main_frequency, encoder_period=50400)

        # For encoders
        self.unwrapper_m1 = Unwrapper(50400)
        self.unwrapper_m2 = Unwrapper(50400)

        # Controllers for motor 1 and 2 in x and y motion
        self.pid_m1_x = PID_pf(1 / main_frequency, 0.008, 3.8171e-04, 2.9608)
        self.pid_m2_x = PID_pf(1 / main_frequency, 0.006, 3.8171e-04, 2.9608)
        self.pid_m1_y = PID_pf(1 / main_frequency, 0.008, 3.8171e-04, 2.9608)
        self.pid_m2_y = PID_pf(1 / main_frequency, 0.006, 3.8171e-04, 2.9608)

        # Kinematics for x and y motion
        self.kinematics_x = Kinematics(0.001)
        self.kinematics_y = Kinematics(0.001)

        # Leds
        self.LED1 = Pin(Pins.LED1, Pin.OUT)
        self.LED2 = Pin(Pins.LED2, Pin.OUT)
        
        return


    def motors_off(self):
        # Do just once
        if self.state_object.is_new_state():
            print('Entered MOTORS OFF')

        # Turn off the motors
        self.motor1.pulse_width_percent(0)
        self.motor2.pulse_width_percent(0)

        return


    def read(self):
        """
        Just when connected to a pc for checking that the emgs are working properly
        """
        # Do just once
        if self.state_object.is_new_state():
            print('Entered READ')

            # Make sure motors are off
            self.motors_off()

            # Leds off
            self.LED1.value(0)
            self.LED2.value(0)

        # Read emg processed values
        emg1_processed_value = self.emg1.process_signal()
        emg2_processed_value = self.emg2.process_signal()
        emg3_processed_value = self.emg3.process_signal()

        # Send signals over serial
        self.serial_pc.set(0,emg1_processed_value)
        self.serial_pc.set(1,emg2_processed_value)
        self.serial_pc.set(2,emg3_processed_value)
        self.serial_pc.send()

        return


    def home(self):
        """
        Allows to control the motors using the potentiometers, for locating the robot in home position
        """
        # Entry action
        if self.state_object.is_new_state():
            print('Entered HOME')

        # Read potentiometers to drive motors directly as a duty cycle, max 50% for limiting velocity
        pot1_value = self.pot1.read() * 2 * 50 / (2**16 - 1) - 50
        pot2_value = self.pot2.read() * 2 * 50 / (2**16 - 1) - 50
        
        # Direction of the motors
        if ((pot1_value > 0 and not self.motor1.direction_pin.value()) or
                (pot1_value < 0 and self.motor1.direction_pin.value())):
            self.motor1.reverse()
        if ((pot2_value > 0 and not self.motor2.direction_pin.value()) or
                (pot2_value < 0 and self.motor2.direction_pin.value())):
            self.motor2.reverse()

        # Activate the motors
        self.motor1.pulse_width_percent(abs(pot1_value))
        self.motor2.pulse_width_percent(abs(pot2_value))

        return


    def x_motion(self):
        """
        Allows to move the robot forwards and backwards
        """
        # Do just once
        if self.state_object.is_new_state():
            print('Entered x_motion')

            # Make sure motors are off
            self.motors_off()

            # Read encoder and unwrap its value
            measured_m1 = self.unwrapper_m1.unwrap(self.motor1.read_encoder_count())
            measured_m2 = self.unwrapper_m2.unwrap(self.motor2.read_encoder_count())

            # Restart unwrapper
            self.unwrapper_m1.reset()
            self.unwrapper_m2.reset()

            # Led 1 on and led 2 off
            self.LED1.value(1)
            self.LED2.value(0)

        # Read emg 1 and 2 
        emg1_processed_value = self.emg1.process_signal()
        emg2_processed_value = self.emg2.process_signal()

        # Reference envelope is emg 1 - emg 2
        reference = emg1_processed_value - emg2_processed_value

        # Set vel to 1 if reference is above a choosen threshold, -1 if below -threshold and0 if it is in middle
        if reference > 10000:
            vel = 1
        elif reference < -10000:
            vel = -1
        else:
            vel = 0

        # Read encoder and unwrap its value
        measured_m1 = self.unwrapper_m1.unwrap(self.motor1.read_encoder_count())
        measured_m2 = self.unwrapper_m2.unwrap(self.motor2.read_encoder_count())

        # Reference from Kinematics, velocity in y is always 0 
        reference_m1, reference_m2 = self.kinematics_x.find_qnew(measured_m1, measured_m2, vel, 0)

        # Prevent the motors to turn more than robot can handdle, given some max counts
        if vel == 1:
            if measured_m1 <= -2200:
                reference_m1 = -2200
            if measured_m2 >= 3000:
                reference_m2 = 3000   
        elif vel == -1:
            if measured_m1 >= 1400:
                reference_m1 = 1400
            if measured_m2 <= -1400:
                reference_m2 = -1400
        else:
            reference_m1 = reference_m1
            reference_m2 = reference_m2

        # Feed reference and measured to PID
        control_output_m1 = self.pid_m1_x.step(
            reference_m1, measured_m1)

        control_output_m2 = self.pid_m2_x.step(
            reference_m2, measured_m2)

        # Set motor direction depending on sign of control_output
        if ((control_output_m1 > 0 and not self.motor1.direction_pin.value()) or
                (control_output_m1 < 0 and self.motor1.direction_pin.value())):
            self.motor1.reverse()

        if ((control_output_m2 > 0 and not self.motor2.direction_pin.value()) or
                (control_output_m2 < 0 and self.motor2.direction_pin.value())):
            self.motor2.reverse()

        # Control motors duty cycle
        duty_cycle_m1 = abs(control_output_m1) * 100 / (self.pid_m1_x.p_gain * 160)
        if duty_cycle_m1 > 100:
            duty_cycle_m1 = 100
        self.motor1.pulse_width_percent(duty_cycle_m1)

        duty_cycle_m2 = abs(control_output_m2) * 100 / (self.pid_m2_x.p_gain * 225)
        if duty_cycle_m2 > 100:
            duty_cycle_m2 = 100
        self.motor2.pulse_width_percent(duty_cycle_m2)

        # Prevent the motors to turn more than robot can handdle (duty cycle)
        if vel == 1:
            if measured_m1 <= -3000:
                self.motor1.pulse_width_percent(0)
            if measured_m2 >= 4000:
                self.motor2.pulse_width_percent(0)      
        elif vel == -1:
            if measured_m1 >= 1400:
                self.motor1.pulse_width_percent(0)
            if measured_m2 <= -1400:
                self.motor2.pulse_width_percent(0) 

        return

    
    def y_motion(self):
        """
        Allows to move the robot up and down
        """
        # Do just once
        if self.state_object.is_new_state():
            print('Entered z_motion')

            # Make sure motors are off
            self.motors_off()

            # Read encoder and unwrap its value
            measured_m1 = self.unwrapper_m1.unwrap(self.motor1.read_encoder_count())
            measured_m2 = self.unwrapper_m2.unwrap(self.motor2.read_encoder_count())

            # Restart unwrapper
            self.unwrapper_m1.reset()
            self.unwrapper_m2.reset()

            # Led 2 on and led 1 off
            self.LED1.value(0)
            self.LED2.value(1)

        # Read emg 1 and 2 
        emg1_processed_value = self.emg1.process_signal()
        emg2_processed_value = self.emg2.process_signal()

        # Reference envelope is emg 1 - emg 2
        reference = emg1_processed_value - emg2_processed_value

        # Set velocity to 1 if reference is above a choosen threshold, -1 if below -threshold and0 if it is in middle
        if reference > 10000:
            vel = 1
        elif reference < -10000:
            vel = -1
        else:
            vel = 0

        # Read encoder and unwrap its value
        measured_m1 = self.unwrapper_m1.unwrap(self.motor1.read_encoder_count())
        measured_m2 = self.unwrapper_m2.unwrap(self.motor2.read_encoder_count())

        # Reference Kinematics, velocity in x is always 0
        reference_m1, reference_m2 = self.kinematics_y.find_qnew(measured_m1, measured_m2, 0, vel)

        # Prevent the motors to turn more than robot can handdle, given some max counts
        max_angle = 3500
        if vel == 1:
            if measured_m1 <= -max_angle:
                reference_m1 = -max_angle
            if measured_m2 >= max_angle:
                reference_m2 = max_angle      
        elif vel == -1:
            if measured_m1 >= max_angle:
                reference_m1 = max_angle
            if measured_m2 <= -max_angle:
                reference_m2 = -max_angle
        else:
            reference_m1 = reference_m1
            reference_m2 = reference_m2


        # Feed reference and measured to PID
        control_output_m1 = self.pid_m1_y.step(
            reference_m1, measured_m1)

        control_output_m2 = self.pid_m2_y.step(
            reference_m2, measured_m2)

        # Set motor direction depending on sign of control_output
        if ((control_output_m1 > 0 and not self.motor1.direction_pin.value()) or
                (control_output_m1 < 0 and self.motor1.direction_pin.value())):
            self.motor1.reverse()

        if ((control_output_m2 > 0 and not self.motor2.direction_pin.value()) or
                (control_output_m2 < 0 and self.motor2.direction_pin.value())):
            self.motor2.reverse()

        # Control motors duty cycle
        duty_cycle_m1 = abs(control_output_m1) * 100 / (self.pid_m1_y.p_gain * 120)
        if duty_cycle_m1 > 100:
            duty_cycle_m1 = 100
        self.motor1.pulse_width_percent(duty_cycle_m1)

        duty_cycle_m2 = abs(control_output_m2) * 100 / (self.pid_m2_y.p_gain * 100)
        if duty_cycle_m2 > 100:
            duty_cycle_m2 = 100
        self.motor2.pulse_width_percent(duty_cycle_m2)

        # Prevent the motors to turn more than robot can handdle (duty cycle)
        if vel == 1:
            if measured_m1 <= -max_angle:
                self.motor1.pulse_width_percent(0)
            if measured_m2 >= max_angle:
                self.motor2.pulse_width_percent(0)      
        elif vel == -1:
            if measured_m1 >= max_angle:
                self.motor1.pulse_width_percent(0)
            if measured_m2 <= -max_angle:
                self.motor2.pulse_width_percent(0) 
        
        return

