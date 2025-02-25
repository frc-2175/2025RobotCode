from pyfrc.tests import *
import rev

# TODO: Test the arm angle limits in constants.py to assert that there are no gaps.

from subsystems.elevatorandarm import ElevatorAndArm
from subsystems.sourceintake import SourceIntake
from subsystems.drivetrain import Drivetrain
import sys

def subsystemCurrentLimits(subsystem):
    currentLimits = []
    for thingname in dir(subsystem):
        if thingname.startswith("__"):
            continue
        thing = getattr(subsystem, thingname)
        if callable(thing):
            continue

        classname = type(getattr(subsystem, thingname)).__name__
        if classname == "SparkMax":
            sparkmax: rev.SparkMax = thing
            currentLimits.append((thingname, sparkmax.configAccessor.getSmartCurrentLimit()))
        elif classname == "SwerveModule":
            return subsystemCurrentLimits(thing)
    return currentLimits

subsystems = [ElevatorAndArm(), SourceIntake(), Drivetrain()]
for subsystem in subsystems:
    currentLimits = subsystemCurrentLimits(subsystem)
    for (name, limit) in currentLimits:
        if limit > 40:
            raise ValueError(f"Spark Max {name} in {type(subsystem).__name__} has an invalid current limit of {limit}. Set one using smartCurrentLimit.")
