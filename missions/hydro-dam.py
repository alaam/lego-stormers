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
    
    def run_in_straight_line(self, hub, motor_pair, motor, distance, speed):
        str_line_runner = RunInStraightLine(hub, motor_pair, motor, distance, speed)
        str_line_runner.run()
        motor_pair.set_stop_action("coast")

    def from_hand_knockoff_energy_units(self, hub, motor_pair, motor):
        turn = GyroTurn(hub, motor_pair, -50)
        turn.turn()

        self.run_in_straight_line(hub, motor_pair, motor, 650, 50)

        turn2 = GyroTurn(hub, motor_pair, -30)
        turn2.turn()

        self.run_in_straight_line(hub, motor_pair, motor, 400, 50)

        self.move_wheels(6, 0, -50)

        turn3 = GyroTurn(hub, motor_pair, -55)
        turn3.turn()

        self.run_in_straight_line(hub, motor_pair, motor, 1800, 50)
    

        

    def run_mission(self):
        motor_a = Motor("A")
        back_motor = Motor("E")

        ## Go from base to middle of board
        self.run_in_straight_line(hub, self.wheels, motor_a, 1700, 50)

        # Turn towards innovation circle
        gyro_turn = GyroTurn(hub, self.wheels, 90)
        gyro_turn.turn()

        ## First go to innovation circle and drop the thingy
        self.run_in_straight_line(hub, self.wheels, motor_a, 590, 50)

        ## Lift up back motor arm to release the thingy
        back_motor.run_for_degrees(280, 60)

        ## From innovation circle go to hand        
        self.run_in_straight_line(hub, self.wheels, motor_a, 560, 50)

        ## Hook the front motor to hand
        self.move_front_motor(-270, 50)

        ## Move back, lifting the hand
        self.move_wheels(5, 0, -20)

        ## Unhook the front motor
        self.move_front_motor(180, 50)

        ## Move back 4 cm
        self.move_wheels(7, 0, -50)

        ## Turn towards oil platform and knockoff energy units, then come home
        self.from_hand_knockoff_energy_units(hub, self.wheels, motor_a)


hand = SmartGrid(MotorPair("A", "B"))

hand.run_mission()