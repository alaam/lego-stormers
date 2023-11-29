# from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
# from spike.control import wait_for_seconds, wait_until, Timer
# from math import *

# hub = PrimeHub()

# hub.light_matrix.show_image('HAPPY')

import sys
import os

class ImportChickys:

    def __init__(self):
        print ("Bismillah")

    def do_chicky(self, slot):
        print("Importing chickys from slot " + str(slot))

        with open("/projects/.slots", "rt") as f:
            slots = eval(str(f.read()))

        #print ("Slots " + str(slots))

        print ("Chickys id = " + str(slots[slot]["id"]))

        in_fname = "/projects/" + str(slots[slot]["id"]) + "/__init__.mpy"

        with open(in_fname, "rb") as f:
            program = f.read()

        out_fname = "/chickys.mpy"

        with open(out_fname, "w") as f:
            f.write(program)

        if "chickys" in sys.modules:
            del sys.modules["chickys"]

        print ("Imported chickys!")

import_chickys = ImportChickys()
import_chickys.do_chicky(19)

raise SystemExit
