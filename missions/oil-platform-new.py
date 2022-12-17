from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from chickys import *

hub = PrimeHub()


class OilPlatform:
    """
    Oil platform mission.
    Slot:
    Alignment:
    """
    def __init__(self, motor_pair):
        self.wheels = motor_pair

    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)

    def move_front_motor(self, amt, speed):
        front_motor = Motor("F")
        front_motor.run_for_degrees(amt, speed)
    
    def start_from_base(self, hub, wheels, motor):
        # first_move = RunInStraightLine(hub, wheels, motor, 100, 20)
        # first_move.run()
        # wheels.set_stop_action("coast")

        # turn = GyroTurn(hub, wheels, -60)
        # turn.turn()

        second_move = RunInStraightLine(hub, wheels, motor, 1180, 40)
        second_move.run()
        wheels.set_stop_action("coast")

        for x in range(0, 3):
            self.move_front_motor(100, 90)
            wait_for_seconds(0.25)
            self.move_front_motor(-90, 40)
        
    def get_tray(self, hub, wheels, motor):
        self.move_wheels(3, 0, -30)

        turn = GyroTurn(hub, wheels, 175)
        turn.turn()
        self.move_wheels(21, 0, -25)

        motor = Motor("E")
        motor.run_for_degrees(-270)

        self.move_wheels(1, 0, -10)

        motor = Motor("E")
        motor.run_for_degrees(-20)

    def speedrun_to_base(self, hub, wheels, motor):
        self.move_wheels(100, 20, 100)

    def run_mission(self):
        motor = Motor("A")
        self.start_from_base(hub, self.wheels, motor)
        self.get_tray(hub, self.wheels, motor)
        self.speedrun_to_base(hub, self.wheels, motor)



oil = OilPlatform(MotorPair("A", "B"))

oil.run_mission()