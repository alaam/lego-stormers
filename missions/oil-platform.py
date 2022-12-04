from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

class OilPlatform:
    def __init__(self, motor_pair):
        self.wheels = motor_pair
    
    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)
    
    def move_front_motor(self, amt, speed):
        front_motor = Motor("F")
        front_motor.run_for_degrees(amt, speed)
    
    def run_mission(self):
        self.move_front_motor(-90, 20)
        self.move_wheels(60, 0, 50)
        # self.move_front_motor(-30, 50)
        self.move_wheels(2, 0, 50)
        for x in range(0, 3):
            self.move_front_motor(100, 50)
            wait_for_seconds(0.25)
            self.move_front_motor(-100, 50)
        self.move_wheels(65, 0, -50)

mission = OilPlatform(MotorPair("A", "B"))

mission.run_mission()