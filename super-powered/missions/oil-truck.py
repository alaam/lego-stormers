from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from chickys import *

hub = PrimeHub()


class OilTruck:
    def __init__(self, motor_pair):
        self.wheels = motor_pair

    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)

    def move_front_motor(self, amt, speed):
        front_motor = Motor("F")
        front_motor.run_for_degrees(amt, speed)

    def run_mission(self):

        motor_a = Motor('A')
        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 960, 30)
        str_line_runner.run()
        self.wheels.set_stop_action('coast')

        self.move_front_motor(-110, 50)
        wait_for_seconds(0.5)
        self.move_wheels(-50, 0, 50)

        #Safety catch
        self.move_front_motor(15, 50)
        wait_for_seconds(0.5)
        self.move_wheels(-50, 0, 50)



        


oil = OilTruck(MotorPair("A", "B"))

oil.run_mission()