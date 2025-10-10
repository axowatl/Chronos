import { Clamp, Clamp01, Rad2Deg } from "./math_functions.js";

export class Vector2
{
    // Constructs a new vector with given x, y components.
    /**
     * 
     * @param {number} x 
     * @param {number} y 
     */
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    // Set x and y components of an existing Vector2.
    /**
     * 
     * @param {number} newX 
     * @param {number} newY 
     */
    Set(newX, newY) { x = newX; y = newY; }

    // Linearly interpolates between two vectors.
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2} b 
     * @param {number} t 
     * @returns {Vector2}
     */
    static Lerp(a, b, t)
    {
        t = Mathf.Clamp01(t);
        return new Vector2(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }

    // Linearly interpolates between two vectors without clamping the interpolant
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2} b 
     * @param {number} t 
     * @returns {Vector2}
     */
    static LerpUnclamped(a, b, t)
    {
        return new Vector2(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }

    // Moves a point /current/ towards /target/.
    /**
     * 
     * @param {Vector2} current 
     * @param {Vector2} target 
     * @param {number} maxDistanceDelta 
     * @returns {Vector2}
     */
    static MoveTowards(current, target, maxDistanceDelta)
    {
        // avoid vector ops because current scripting backends are terrible at inlining
        const toVector_x = target.x - current.x;
        const toVector_y = target.y - current.y;

        const sqDist = toVector_x * toVector_x + toVector_y * toVector_y;

        if (sqDist == 0 || (maxDistanceDelta >= 0 && sqDist <= maxDistanceDelta * maxDistanceDelta))
            return target;

        const dist = Math.sqrt(sqDist);

        return new Vector2(current.x + toVector_x / dist * maxDistanceDelta,
            current.y + toVector_y / dist * maxDistanceDelta);
    }

    // Multiplies two vectors component-wise.
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2} b 
     * @returns {Vector2}
     */
    static Scale(a, b) { return new Vector2(a.x * b.x, a.y * b.y); }

    // Multiplies every component of this vector by the same component of /scale/.
    /**
     * 
     * @param {Vector2} scale 
     */
    Scale(scale) { x *= scale.x; y *= scale.y; }

    // Makes this vector have a ::ref::magnitude of 1.
    Normalize()
    {
        const mag = magnitude;
        if (mag > kEpsilon)
            this = this / mag;
        else
            this = zero;
    }

    // Returns this vector with a ::ref::magnitude of 1 (RO).
    get normalized()
    {
        let v = new Vector2(x, y);
        v.Normalize();
        return v;
    }

    get ToString()
    {
        retrun `(${this.x}, ${this.y})`;
    }

    /**
     * 
     * @param {Vector2} other 
     * @returns {boolean}
     */
    Equals(other)
    {
        return x == other.x && y == other.y;
    }

    /**
     * 
     * @param {Vector2} inDirection 
     * @param {Vector2} inNormal 
     * @returns {Vector2}
     */
    static Reflect(inDirection, inNormal)
    {
        const factor = -2 * Vector2.Dot(inNormal, inDirection);
        return new Vector2(factor * inNormal.x + inDirection.x, factor * inNormal.y + inDirection.y);
    }

    /**
     * 
     * @param {Vector2} inDirection 
     * @returns {Vector2}
     */
    static Perpendicular(inDirection)
    {
        return new Vector2(-inDirection.y, inDirection.x);
    }

    // Dot Product of two vectors.
    /**
     * 
     * @param {Vector2} lhs 
     * @param {Vector2} rhs 
     * @returns {number}
     */
    static Dot(lhs, rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }

    // Returns the length of this vector (RO).
    get magnitude() { return Math.sqrt(x * x + y * y); }
    // Returns the squared length of this vector (RO).
    get sqrMagnitude() { return x * x + y * y; }

    // Returns the angle in degrees between /from/ and /to/.
    /**
     * 
     * @param {Vector2} from 
     * @param {Vector2} to 
     * @returns {number}
     */
    static Angle(from, to)
    {
        // sqrt(a) * sqrt(b) = sqrt(a * b) -- valid for real numbers
        const denominator = Math.sqrt(from.sqrMagnitude * to.sqrMagnitude);
        if (denominator < kEpsilonNormalSqrt)
            return 0;

        const dot = Clamp(Dot(from, to) / denominator, -1, 1);
        return Math.acos(dot) * Rad2Deg;
    }

    // Returns the signed angle in degrees between /from/ and /to/. Always returns the smallest possible angle
    /**
     * 
     * @param {Vector2} from 
     * @param {Vector2} to 
     * @returns {number}
     */
    static SignedAngle(from, to)
    {
        const unsigned_angle = Vector2.Angle(from, to);
        const sign = Math.sign(from.x * to.y - from.y * to.x);
        return unsigned_angle * sign;
    }

    // Returns the distance between /a/ and /b/.
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2} b 
     * @returns {number}
     */
    static Distance(a, b)
    {
        const diff_x = a.x - b.x;
        const diff_y = a.y - b.y;
        return Math.sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    // Returns a copy of /vector/ with its magnitude clamped to /maxLength/.
    /**
     * 
     * @param {Vector2} vector 
     * @param {number} maxLength 
     * @returns {Vector2}
     */
    static ClampMagnitude(vector, maxLength)
    {
        const sqrMagnitude = vector.sqrMagnitude;
        if (sqrMagnitude > maxLength * maxLength)
        {
            const mag = Math.sqrt(sqrMagnitude);

            //these intermediate variables force the intermediate result to be
            //of float precision. without this, the intermediate result can be of higher
            //precision, which changes behavior.
            const normalized_x = vector.x / mag;
            const normalized_y = vector.y / mag;
            return new Vector2(normalized_x * maxLength,
                normalized_y * maxLength);
        }
        return vector;
    }

    // Returns a vector that is made from the smallest components of two vectors.
    /**
     * 
     * @param {Vector2} lhs
     * @param {Vector2} rhs
     * @returns {Vector2}
     */
    static Min(lhs, rhs) { return new Vector2(Math.min(lhs.x, rhs.x), Math.min(lhs.y, rhs.y)); }

    // Returns a vector that is made from the largest components of two vectors.
    /**
     * 
     * @param {Vector2} lhs 
     * @param {Vector2} rhs 
     * @returns {Vector2}
     */
    static Max(lhs, rhs) { return new Vector2(Math.max(lhs.x, rhs.x), Math.max(lhs.y, rhs.y)); }

    /**
     * 
     * @param {Vector2} current 
     * @param {Vector2} target 
     * @param {Vector2} currentVelocity 
     * @param {number} smoothTime 
     * @param {number} maxSpeed 
     * @param {number} deltaTime 
     * @returns {Vector2}
     */
    static SmoothDamp(current, target, currentVelocity, smoothTime, maxSpeed = Infinity, deltaTime = 0.016)
    {
        // Based on Game Programming Gems 4 Chapter 1.10
        smoothTime = Math.max(0.0001, smoothTime);
        const omega = 2 / smoothTime;

        const x = omega * deltaTime;
        const exp = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);

        const change_x = current.x - target.x;
        const change_y = current.y - target.y;
        const originalTo = target;

        // Clamp maximum speed
        const maxChange = maxSpeed * smoothTime;

        const maxChangeSq = maxChange * maxChange;
        const sqDist = change_x * change_x + change_y * change_y;
        if (sqDist > maxChangeSq)
        {
            var mag = Math.sqrt(sqDist);
            change_x = change_x / mag * maxChange;
            change_y = change_y / mag * maxChange;
        }

        target.x = current.x - change_x;
        target.y = current.y - change_y;

        const temp_x = (currentVelocity.x + omega * change_x) * deltaTime;
        const temp_y = (currentVelocity.y + omega * change_y) * deltaTime;

        currentVelocity.x = (currentVelocity.x - omega * temp_x) * exp;
        currentVelocity.y = (currentVelocity.y - omega * temp_y) * exp;

        let output_x = target.x + (change_x + temp_x) * exp;
        let output_y = target.y + (change_y + temp_y) * exp;

        // Prevent overshooting
        const origMinusCurrent_x = originalTo.x - current.x;
        const origMinusCurrent_y = originalTo.y - current.y;
        const outMinusOrig_x = output_x - originalTo.x;
        const outMinusOrig_y = output_y - originalTo.y;

        if (origMinusCurrent_x * outMinusOrig_x + origMinusCurrent_y * outMinusOrig_y > 0)
        {
            output_x = originalTo.x;
            output_y = originalTo.y;

            currentVelocity.x = (output_x - originalTo.x) / deltaTime;
            currentVelocity.y = (output_y - originalTo.y) / deltaTime;
        }
        return new Vector2(output_x, output_y);
    }

    // Adds two vectors.
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2} b 
     * @returns {Vector2}
     */
    static add(a, b) { return new Vector2(a.x + b.x, a.y + b.y); }
    // Subtracts one vector from another.
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2} b 
     * @returns {Vector2}
     */
    static subtract(a, b) { return new Vector2(a.x - b.x, a.y - b.y); }
    // Multiplies one vector by another.
    /**
     * 
     * @param {Vector2 | number} a 
     * @param {Vector2 | number} b 
     * @returns {Vector2}
     */
    static multiply(a, b) {
        const _aIsVec = a instanceof Vector2;
        const _bIsVec = b instanceof Vector2;
        if (_aIsVec && _bIsVec)
            return new Vector2(a.x * b.x, a.y * b.y);
        else if (_aIsVec)
            return new Vector2(a.x * b, a.y * b);
        else
            return new Vector2(a * b.x, a * b.y);
    }
    // Divides one vector over another.
    /**
     * 
     * @param {Vector2} a 
     * @param {Vector2 | number} b 
     * @returns {Vector2}
     */
    static divide(a, b) {
        if (b instanceof Vector2)
            return new Vector2(a.x / b.x, a.y / b.y);
        else
            return new Vector2(a.x / b, a.y / b);
    }
    // Negates a vector.
    /**
     * 
     * @param {Vector2} a 
     * @returns {Vector2}
     */
    static negate(a) { return new Vector2(-a.x, -a.y); }
    // Returns true if the vectors are equal.
    /**
     * 
     * @param {Vector2} lhs 
     * @param {Vector2} rhs 
     * @returns {boolean}
     */
    static equals(lhs, rhs)
    {
        // Returns false in the presence of NaN values.
        const diff_x = lhs.x - rhs.x;
        const diff_y = lhs.y - rhs.y;
        return (diff_x * diff_x + diff_y * diff_y) < kEpsilon * kEpsilon;
    }

    // Returns true if vectors are different.
    /**
     * 
     * @param {Vector2} lhs 
     * @param {Vector2} rhs 
     * @returns {boolean}
     */
    static notEquals(lhs, rhs)
    {
        // Returns true in the presence of NaN values.
        return !(Vector2.equals(lhs, rhs));
    }

    static get zero() { new Vector2(0, 0); }
    static get one() { new Vector2(1, 1); }
    static get up() { new Vector2(0, 1); }
    static get down() { new Vector2(0, -1); }
    static get left() { new Vector2(-1, 0); }
    static get right() { new Vector2(1, 0); }
    static get positiveInfinity() { new Vector2(Infinity, Infinity); }
    static get negativeInfinity() { new Vector2(-Infinity, -Infinity); }

    // *Undocumented*
    kEpsilon = 0.00001;
    // *Undocumented*
    kEpsilonNormalSqrt = 1e-15;
}
