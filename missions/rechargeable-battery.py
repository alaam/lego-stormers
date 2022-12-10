from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()


class battery:
    """
    Rechargable battery mission
    Slot: 10
    Base: Right
    Alignment: 3 from the left edge
    """
    def __init__(self, motor_pair):
        self.wheels=motor_pair
    
    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)
    
    def run_mission(self):
        self.move_wheels(50, -6, 70)

        self.move_wheels(13, -52, 50)

        self.move_wheels(30, 0, -70)

        self.move_wheels(15, -90, 50)

        self.move_wheels(30, 0, 70)






battery = battery(MotorPair("A", "B"))
battery.run_mission()