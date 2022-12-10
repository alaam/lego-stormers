from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from chickys import *

hub = PrimeHub()

class SmartGrid:
    """
    Smart grid mission. Attachment is a claw that grabs the base and pulls
    Slot: 5
    Alignment:
    """
    def __init__(self,motor_pair):
        self.wheels=motor_pair

    def move_wheels(self,amt,steering,speed):
        self.wheels.move(amt,"cm",steering,speed)
    
    def move_front_motor(self, amt, speed):
        front_motor = Motor("F")
        front_motor.run_for_degrees(amt, speed)
    
    def run_mission(self):
        motor_a = Motor("A")
        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 200)
        str_line_runner.run()
        self.wheels.set_stop_action("coast")

        gyro_turn = GyroTurn(hub, self.wheels, -85)
        gyro_turn.turn()

        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 1600)
        str_line_runner.run()
        self.wheels.set_stop_action("coast")

        gyro_turn = GyroTurn(hub, self.wheels, 86)
        gyro_turn.turn()

        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 1250)
        str_line_runner.run()
        self.wheels.set_stop_action("coast")

        self.move_front_motor(-110, 50)
        
        self.move_wheels(5, 0, -30)

        self.move_front_motor(130, 50)

        self.move_wheels(52, 0, -50)

        gyro_turn = GyroTurn(hub, self.wheels, 105)
        gyro_turn.turn()

        self.move_wheels(100, 6, 50)


hand = SmartGrid(MotorPair("A", "B"))

hand.run_mission()