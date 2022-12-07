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
        #Aligment: 3rd from the second black line
        hub.motion_sensor.reset_yaw_angle()
        motor = Motor("A")
        motor.set_degrees_counted(0)
        self.wheels.set_stop_action("hold")
        while True:
            d_counted = motor.get_degrees_counted()
            yaw_angle = hub.motion_sensor.get_yaw_angle()
            self.wheels.start(0-yaw_angle, 30)
            if abs(d_counted) >= 1140:
                break
        self.wheels.stop()

        self.move_front_motor(-110, 50)

        self.move_wheels(4,0, 50)

        for x in range(0,4):
            self.move_front_motor(120,50)
            wait_for_seconds(0.25)
            self.move_front_motor(-120,50)
        
        self.move_wheels(80,0,-50)




oil = OilPlatform(MotorPair("A", "B"))

oil.run_mission()