from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

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
    def __init__(self, hub, motors, motor, degrees_counted):
        self.hub = hub
        self.motors = motors
        self.motor = motor
        self.degrees_counted = degrees_counted

    def run(self):
        # Reset yaw angle first
        hub.motion_sensor.reset_yaw_angle()
        # Reset degrees counted to 0, since we are going to use this value to measure distance
        self.motor.set_degrees_counted(0)
        # This seems to be necessary for stop action to take affect
        self.motors.stop()
        # Hold seems better than coast or brake to ensure smooth stop action
        self.motors.set_stop_action("hold")
        while True:
            d_counted = self.motor.get_degrees_counted()
            yaw_angle = self.hub.motion_sensor.get_yaw_angle()
            self.motors.start((0 - yaw_angle)*2, 40)
            #print ('Yaw ' + str(yaw_angle) + ' dc ' + str(d_counted))
            if abs(d_counted) >= self.degrees_counted:
                break
        self.motors.set_stop_action('coast')
        self.motors.stop()
        print ('done')



class HybridCar:
    def __init__(self, motor_pair):
        self.wheels = motor_pair

    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)
    
    def move_front_motor(self, amt, speed):
        front_motor = Motor("F")
        front_motor.run_for_degrees(amt, speed)

    def run_mission(self):
        # self.move_wheels(30, 0, 50)
        motor_a = Motor('A')
        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 360)
        str_line_runner.run()
        self.wheels.set_stop_action('coast')


        self.move_wheels(10, -47, 50)

        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 1500)
        str_line_runner.run()
        self.wheels.set_stop_action('coast')

        self.move_front_motor(-150, 20)

        self.move_wheels(85, -5, -50)


hybrid_car = HybridCar(MotorPair("A", "B"))

hybrid_car.run_mission()