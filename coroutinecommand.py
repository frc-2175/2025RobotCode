#stealing from 4096
from functools import wraps
from typing import Callable, Iterator, ParamSpec, Set

from commands2 import Command, CommandScheduler
from commands2.subsystem import Subsystem

import ntutil


P = ParamSpec("P")

commandLog = ntutil.getStringLog("/Log/Commands")

def commandify(func: Callable[P, Iterator]) -> Callable[P, Command]:
    @wraps(func)
    def wrapper(*args: P.args, **kwargs: P.kwargs):
        gen = func(*args, **kwargs)

        class GeneratorCommand(Command):
            def __init__(self) -> None:
                super().__init__()
                self.initialized = False
                self.is_finished = False

                # Modified from 4096. We don't care about requirements.
                # r: "Robot" = args[0]  # type: ignore
                # self.addRequirements(r.subsystems)

            def initialize(self):
                commandLog.append(f"{self.getName()}: Initializing")
                if self.initialized:
                    commandLog.append("ERROR: Initialized a GeneratorCommand more than once! This will not work; instead, the generator will just resume partway through!")
                self.initialized = True

            def execute(self) -> None:
                try:
                    next(gen)
                except StopIteration:
                    self.is_finished = True

            def isFinished(self) -> bool:
                finished = self.is_finished
                if finished:
                    commandLog.append(f"{self.getName()}: Finished!")
                return finished

            def getName(self) -> str:
                return func.__name__

            def __iter__(self):
                return gen

        return GeneratorCommand()

    return wrapper


class RestartableCommand(Command):
    def __init__(self, commandCreator: Callable[[], Command]):
        self.make = commandCreator
        self.cmd = self.make()
        self.didEverInitialize = False
        self.isActive = False

    def initialize(self):
        if self.isActive:
            CommandScheduler.getInstance().cancel(self.cmd)
        self.cmd = self.make()
        if self.didEverInitialize:
            finishedMsg = " (had not finished)" if self.isActive else ""
            commandLog.append(f"RestartableCommand({self.cmd.getName()}): Restarting!{finishedMsg}")
        self.didEverInitialize = True
        self.isActive = True
        return self.cmd.initialize()

    def execute(self):
        return self.cmd.execute()

    def isFinished(self) -> bool:
        return self.cmd.isFinished()

    def end(self, interrupted: bool):
        self.isActive = False
        return self.cmd.end(interrupted)

    def getRequirements(self) -> Set[Subsystem]:
        return self.cmd.getRequirements()
