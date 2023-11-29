from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from chickys import *

hub = PrimeHub()

#Class for Windturbine:07
#Class for TV:08
class LegoStormers_Missions_07_08:
    """
    TV and Windturbine. This class combines the two missions.
    Slot: 4
    Alignment:
    """
    def __init__(self, motor_pair):
        #Initializing the program
        self.wheels = motor_pair

    """
    Method for moving wheels
    We are using cm as units
    """
    def move_wheels(self, amt, steering, speed):
        self.wheels.move(amt, "cm", steering, speed)

    def push_and_move_back(self, dist, steering, speed):
        """
            Moves the robot forward and moves back the same distance
            Keywords:
            dist: amount of movemnt; in centimeters
            steering: the angle of movement
            speed: the speed of the robot when moving
        """
        self.move_wheels(dist, steering, speed)
        self.move_wheels(dist, steering, speed * -1)

    def run_mission(self):
        #All the movements for the program to run
        motor_a = Motor("A")
        back_Motor = Motor("E")


        #str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 650, 50)
        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 680, 50)
        str_line_runner.run()
        self.wheels.set_stop_action("coast")

        wait_for_seconds(0.5)

        # self.move_wheels(45, 0, 65)
        
        self.move_wheels(10, 0, -50)

        turn = GyroTurn(hub, self.wheels, -45)
        turn.turn()

        self.move_wheels(35, 0, 40)

        turn = GyroTurn(hub, self.wheels, 95)
        turn.turn()

        self.move_wheels(14, 0, -40)

        back_Motor.run_for_degrees(-300, 50)

        back_Motor.run_for_degrees(90, 50)
        self.move_wheels(1, 0, -10)
        back_Motor.run_for_degrees(-90, 50)

        str_line_runner = RunInStraightLine(hub, self.wheels, motor_a, 300, 50)
        str_line_runner.run()
        self.wheels.set_stop_action("coast")


        for x in range(0, 3):
            self.move_wheels(25, 0, 40)
            wait_for_seconds(0.25)
            self.move_wheels(10, 0, -40)
        
        self.move_wheels(5, 0, -40)
        turn = GyroTurn(hub, self.wheels, -80)
        turn.turn()
        self.move_wheels(90, 0, -100)



#Creating an instance of the class
missions_07_08 = LegoStormers_Missions_07_08(MotorPair("A", "B"))

#Starting the run method
missions_07_08.run_mission()