
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from chickys import *


hub = PrimeHub()

class OilTruckDropOff:
    """
    Oil truck drop off. This mission combines oil truck drop off and pickup rechargable battery
    Slot:
    Alignment:
    """
    def __init__(self, motor_pair):
        self.wheels = motor_pair

    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)
    
    def turn(self, angle):
        hub.motion_sensor.reset_yaw_angle()
        while abs(hub.motion_sensor.get_yaw_angle()) < abs(angle):
            if angle < 0:
                self.wheels.start_tank(0, 30)
            elif angle > 0:
                self.wheels.start_tank(30, 0)
        self.wheels.stop()
    

    def run_mission(self):

        motor_a = Motor('A')
        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 300, 40)
        str_line_runner.run()
        self.wheels.set_stop_action('coast')

        #self.move_wheels(10, -51, 35)
        self.turn(-31)

        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 2875,50)
        str_line_runner.run()
        self.wheels.set_stop_action('coast')

        ## Alternatively
        ## Try to collect battery pack
        self.move_wheels(22, 0, -50)        
        self.turn(85) 
        self.move_wheels(90, 5, 50)
        
oil_tank_dropoff = OilTruckDropOff(MotorPair("A", "B"))
oil_tank_dropoff.run_mission()