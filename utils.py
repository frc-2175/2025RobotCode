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
