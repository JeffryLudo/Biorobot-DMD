
from state_machine import Robot
from timer_definitions import Timers

# Start the timer 2 with a frequecny for the main loop of 200 Hz
ticker_number = Timers.RUN
main_frequency = 200

robot = Robot(ticker_number, main_frequency)
robot.start()




