import math

def lerp(a: float, b: float, t: float) -> float:
    return (1-t) * a + t * b

def clamp(value: float, min: float, max: float) -> float:
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value

def remap(value: float, range1: tuple[float, float], range2: tuple[float, float]) -> float:
    t = (value - range1[0]) / (range1[1] - range1[0])
    return lerp(range2[0], range2[1], t)

def sign(v: float) -> float:
    if v < 0:
        return -1
    else:
        return 1

# Implementation borrowed from team 3206.
def stepTowardsCircular(current:float, target:float, stepSize:float) -> float:
    current = wrapAngle(current)
    target = wrapAngle(target)
    stepDirection = sign(target - current)
    difference = abs(current - target)
    if difference <= stepSize:
        return target
    elif difference > math.pi:
        if current + 2 * math.pi - target < stepSize or target + 2 *  math.pi - current < stepSize:
            return target
        else:
            return wrapAngle(current - stepDirection * stepSize)
    else:
        return current + stepDirection * stepSize
    
# Implementation borrowed from team 3206.
def wrapAngle(angle:float)  -> float:
    if angle == 2 * math.pi:
        return 0
    elif angle > 2 * math.pi:
        return angle - 2 * math.pi * math.floor(angle / (2* math.pi))
    elif angle < 0:
        return angle + 2 * math.pi * (math.floor(-angle / (2* math.pi)) + 1)
    else:
        return angle

def angleDifference(angleA:float, angleB:float) -> float:
    difference = abs(angleA - angleB)
    if difference > math.pi:
        return (2 * math.pi) - difference
    else:
        return difference
    
class RotationSlewRateLimiter:
    current = 0

    def __init__(self, rateLimit: float): # radians per second
        self.rateLimit = rateLimit
    
    def calculate(self, target: float, force: bool = False): # radians
        if force:
            new = target
        else:
            new = stepTowardsCircular(self.current, target, self.rateLimit / 50) # assuming 50hz robot loop
        self.current = new
        return new

    def setRateLimit(self, rateLimit: float): # rad/s
        self.rateLimit = rateLimit
