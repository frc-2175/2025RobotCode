from pyfrc.tests import *

import sys
import pkgutil
import importlib
import inspect

import rev

import subsystems

# TODO: Test the arm angle limits in constants.py to assert that there are no gaps.

def get_subsystem_classes():
    classes = []
    
    # Iterate through all modules in the subsystems package
    for _, module_name, _ in pkgutil.iter_modules(subsystems.__path__, subsystems.__name__ + "."):
        module = importlib.import_module(module_name)
        
        # Find the single class in the module
        class_members = [cls for _, cls in inspect.getmembers(module, inspect.isclass) if cls.__module__ == module_name]
        
        if len(class_members) == 1:
            classes.append(class_members[0])
        else:
            print(f"Warning: {module_name} has {len(class_members)} classes instead of 1")

    return classes

def subsystem_current_limits(subsystem):
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
            return subsystem_current_limits(thing)
    return currentLimits

subsystem_classes = get_subsystem_classes()
for subsystem_class in subsystem_classes:
    subsystem = subsystem_class()
    currentLimits = subsystem_current_limits(subsystem)
    for (name, limit) in currentLimits:
        if limit > 40:
            raise ValueError(f"Spark Max {name} in {type(subsystem).__name__} has an invalid current limit of {limit}. Set one using smartCurrentLimit.")
