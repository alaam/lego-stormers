from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from chickys import *

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
        #Aligment: 3rd from the second black line
        hub.motion_sensor.reset_yaw_angle()
        motor = Motor("A")
        str_runner = RunInStraightLine(hub, self.wheels, motor, 1140, 45)
        str_runner.run()
        print ("Str line is done, moving arm down")
        self.move_front_motor(-110, 50)
        print ("Arm down, moving forward")
        self.move_wheels(4,0, 50)
        print ("Pumping oil")
        for x in range(0,3):
            self.move_front_motor(120,50)
            wait_for_seconds(0.25)
            self.move_front_motor(-120,50)

        self.move_wheels(70,0,-60)




oil = OilPlatform(MotorPair("A", "B"))

oil.run_mission()