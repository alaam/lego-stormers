from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()


class Phil:
    def __init__(self,motor_pair):
        self.wheels=motor_pair

    def move_wheels(self,amt,steering,speed):
        self.wheels.move(amt,"cm",steering,speed)

    def run_mission(self):
        # wait_for_seconds(3)
        self.move_wheels(220,0,100)

phil=Phil(MotorPair("A","B"))
phil.run_mission()