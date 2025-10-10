export function Clamp01(value)
{
    if (value < 0)
        return 0;
    else if (value > 1)
        return 1;
    else
        return value;
}

export function Clamp(value, min, max)
{
    if (value < min)
        value = min;
    else if (value > max)
        value = max;
    return value;
}

// Degrees-to-radians conversion constant (RO).
export const Deg2Rad = Math.PI * 2 / 360;

// Radians-to-degrees conversion constant (RO).
export const Rad2Deg = 1 / Deg2Rad;