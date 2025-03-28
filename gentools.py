from functools import wraps
from typing import Callable, Generator, Generic, ParamSpec, TypeVar

P = ParamSpec("P")
Y = TypeVar("Y")
S = TypeVar("S")
R = TypeVar("R")

class GeneratorWrapper(Generic[Y, S, R]):
    def __init__(self, gen: Generator[Y, S, R]) -> None:
        self.done = False
        self.gen = gen
    def __iter__(self):
        return self
    def __next__(self):
        try:
            return next(self.gen)
        except StopIteration:
            self.done = True
            raise StopIteration

    def resume(self):
        """Calls next but will never raise StopIteration."""
        return next(self, None)

def doneable(func: Callable[P, Generator[Y, S, R]]) -> Callable[P, GeneratorWrapper[Y, S, R]]:
    @wraps(func)
    def makeWrapper(*args: P.args, **kwargs: P.kwargs):
        return GeneratorWrapper(func(*args, **kwargs))
    return makeWrapper
