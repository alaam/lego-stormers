from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

class RunInStraightLine:
    '''
    This class is a straight line algorithm, that makes the robot travel in a straight line correcting for drift 
    caused by uneven surface, charge or motor differences. We used the yaw angle and calculated the correction angle. 
    Since we are using start() function of the MotorPair, we cannot use distance so we are using degrees counted by a single motor for distance.
    Args:
        hub: The hub
        motors: The MotorPair to 
        motor: One of the motors in the MotorPair. We used the degrees_counted value of this motor for stop condition
        degrees_counted (int): Number of degrees turned by the motor during a run of this program
    '''
    def __init__(self, hub, motors, motor, degrees_counted, speed=40):
        self.hub = hub
        self.motors = motors
        self.motor = motor
        self.degrees_counted = degrees_counted
        self.speed = speed
    
    def run(self):
        # Reset yaw angle first
        self.hub.motion_sensor.reset_yaw_angle()
        # Reset degrees counted to 0, since we are going to use this value to measure distance
        self.motor.set_degrees_counted(0)
        # This seems to be necessary for stop action to take affect
        self.motors.stop()
        # Hold seems better than coast or brake to ensure smooth stop action
        self.motors.set_stop_action("hold")
        while True:
            d_counted = self.motor.get_degrees_counted()
            yaw_angle = self.hub.motion_sensor.get_yaw_angle()
            self.motors.start((0 - yaw_angle) * 2, self.speed)
            #print ('Yaw ' + str(yaw_angle) + ' dc ' + str(d_counted))
            if abs(d_counted) >= self.degrees_counted:
                break
        self.motors.set_stop_action('coast')
        self.motors.stop()
        print ('done')

class Greet:

    def __init__(self):
        pass
    
    def say_bismillah(self):
        print ("Bismillah!")

    def say_alhamdulillah(self):
        print ("Alhamdulillah")


class GyroTurn:
    """
    Implements turns using gyro sensor
    """
    def __init__(self, hub, wheels, angle):
        self.hub = hub
        self.wheels = wheels
        self.angle = angle
    
    def turn(self):
        self.hub.motion_sensor.reset_yaw_angle()
        while abs(self.hub.motion_sensor.get_yaw_angle()) < abs(self.angle):
            if self.angle < 0:
                self.wheels.start_tank(0, 30)
            elif self.angle > 0:
                self.wheels.start_tank(30, 0)
        self.wheels.stop()