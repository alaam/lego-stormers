from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

hub.light_matrix.show_image('HAPPY')
import time

class PositionMotor:
    '''
    Use this class to position the attachements of the motor at a certain location.
    Left button moves the motor up by turn_degrees, right moves it down.
    Args:
        hub: The hub
        motor: The motor to turn
        turn_degrees (int): Degrees to turn the motor, default 20 degrees
        timeout (int): The program terminates after this timeout
    '''
    def __init__(self, hub, motor, turn_degrees = 20, timeout = 5):
        self.hub = hub
        self.motor = motor
        self.turn_degrees = turn_degrees
        self.timeout = timeout

    def run(self):
        st = time.time()
        while True:
            now = time.time()
            if (now - st >= self.timeout):
                print ('Now ' + str(now) + ' St ' + str(st) + ' Diff ' + str(now -st))
                break
            if self.hub.left_button.was_pressed():
                self.motor.run_for_degrees(self.turn_degrees)
                st = now
            elif self.hub.right_button.was_pressed():
                self.motor.run_for_degrees(self.turn_degrees * -1)
                st = now
        print ('Done')


position_motors = PositionMotor(hub, Motor('F'), 30, 5)
position_motors.run()