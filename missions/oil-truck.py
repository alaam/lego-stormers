from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

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
        self.move_wheels(20, 0, 40)
        self.move_front_motor(-320, 50)
        self.move_wheels(-60, 0, 60)
        

oil = OilTruck(MotorPair("A", "B"))

oil.run_mission()