from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

class InnovationProject:
    def __init__(self, motor_pair):
        self.wheels = motor_pair
    
    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)
    
    def run_mission(self):
        self.move_wheels(99, -10, 70)
        self.move_wheels(105,-10,-70)
    


run = InnovationProject(MotorPair("A", "B"))

run.run_mission()