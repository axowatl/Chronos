//#region Math
/// 2D vector
/// This can be used to represent a point or free vector
export class Vec2 {
    constructor(x, y) {
        this.x = x
        this.y = y
    }
}

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
export class CosSin {
    constructor(c, s) {
        this.cosine = c
        this.sine = s
    }
}

/// 2D rotation
/// This is similar to using a complex number for rotation
export class Rot {
    constructor(c, s) {
        this.c = c
        this.s = s
    }
}

/// A 2D rigid transform
export class Transform {
    constructor(p, q) {
        this.p = p
        this.q = q
    }
}

/// A 2-by-2 Matrix
export class Mat22 {
    constructor(cx, cy) {
        this.cx = cx
        this.cy = cy
    }
}

/// Axis-aligned bounding box
export class AABB {
    constructor(lb, ub) {
        this.lowerBound = lb
        this.upperBound = ub
    }
}

/// separation = dot(normal, point) - offset
export class Plane {
    constructor(n, o) {
        this.normal = n
        this.offset = o
    }
}
// #endregion Math
//#region Math
/// https://en.wikipedia.org/wiki/Pi
export const PI = 3.14159265359

export const Vec2_zero = new Vec2(0, 0);
export const Rot_identity = new Rot(1, 0);
export const Transform_identity = new Transform(
    new Vec2(0, 0),
    new Rot(1, 0)
);
export const Mat22_zero = new Mat22(new Vec2(0, 0), new Vec2(0, 0));

/// Is this a valid number? Not NaN or infinity.
export function IsValidFloat(a) {
    if (isNaN(a)) {
        return false
    }

    if (isFinite(a)) {
        return false
    }

    return true
}

/// Is this a valid vector? Not NaN or infinity.
export function IsValidVec2(v) {
    if (isNaN(v.x) || isNaN(v.y)) {
        return false
    }

    if (isFinite(v.x) || isFinite(v.y)) {
        return false
    }

    return true
}

/// Is this a valid rotation? Not NaN or infinity. Is normalized.
export function IsValidRotation(q) {
    if (isNaN(q.s) || isNaN(q.c)) {
        return false
    }

    if (isFinite(q.s) || isFinite(q.c)) {
        return false
    }

    return IsNormalizedRot(q)
}

/// Is this a valid transform? Not NaN or infinity. Rotation is normalized.
export function IsValidTransform(t) {
    if (IsValidVec2(t.p) == false) {
        return false
    }

    return IsValidRotation(t.q)
}

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
export function IsValidPlane(a) {
    return (
        IsValidVec2(a.normal) &&
        IsNormalized(a.normal) &&
        IsValidFloat(a.offset)
    )
}

/// @return a float clamped between a lower and upper bound
export function ClampFloat(a, lower, upper) {
    return a < lower ? lower : a > upper ? upper : a
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
export function Atan2(y, x) {
    // Added check for (0,0) to match atan2f and avoid NaN
    if (x == 0 && y == 0) {
        return 0
    }

    const ax = Math.abs(x)
    const ay = Math.abs(y)
    const mx = Math.max(ay, ax)
    const mn = Math.min(ay, ax)
    const a = mn / mx

    // Minimax polynomial approximation to atan(a) on [0,1]
    const s = a * a
    const c = s * a
    const q = s * s
    let r = 0.024840285 * q + 0.18681418
    const t = -0.094097948 * q - 0.33213072
    r = r * s + t
    r = r * c + a

    // Map to full circle
    if (ay > ax) {
        r = 1.57079637 - r
    }

    if (x < 0) {
        r = 3.14159274 - r
    }

    if (y < 0) {
        r = -r
    }

    return r
}

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
export function ComputeCosSin(radians) {
    const x = UnwindAngle(radians)
    const pi2 = PI * PI

    // cosine needs angle in [-pi/2, pi/2]
    let c
    if (x < -0.5 * PI) {
        const y = x + PI
        const y2 = y * y
        c = -(pi2 - 4 * y2) / (pi2 + y2)
    } else if (x > 0.5 * PI) {
        const y = x - PI
        const y2 = y * y
        c = -(pi2 - 4 * y2) / (pi2 + y2)
    } else {
        const y2 = x * x
        c = (pi2 - 4 * y2) / (pi2 + y2)
    }

    // sine needs angle in [0, pi]
    let s
    if (x < 0) {
        const y = x + PI
        s = (-16 * y * (PI - y)) / (5 * pi2 - 4 * y * (PI - y))
    } else {
        s = (16 * x * (PI - x)) / (5 * pi2 - 4 * x * (PI - x))
    }

    const mag = Math.sqrt(s * s + c * c)
    const invMag = mag > 0 ? 1 / mag : 0
    const cs = new CosSin(c * invMag, s * invMag)
    return cs
}

/// Vector dot product
export function Dot(a, b) {
    return a.x * b.x + a.y * b.y
}

/// Vector cross product. In 2D this yields a scalar.
export function Cross(a, b) {
    return a.x * b.y - a.y * b.x
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
export function CrossVS(v, s) {
    return new Vec2(s * v.y, -s * v.x)
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
export function CrossSV(s, v) {
    return new Vec2(-s * v.y, s * v.x)
}

/// Get a left pointing perpendicular vector. Equivalent to CrossSV(1.0f, v)
export function LeftPerp(v) {
    return new Vec2(-v.y, v.x)
}

/// Get a right pointing perpendicular vector. Equivalent to CrossVS(v, 1.0f)
export function RightPerp(v) {
    return new Vec2(v.y, -v.x)
}

/// Vector addition
export function Add(a, b) {
    return new Vec2(a.x + b.x, a.y + b.y)
}

/// Vector subtraction
export function Sub(a, b) {
    return new Vec2(a.x - b.x, a.y - b.y)
}

/// Vector negation
export function Neg(a) {
    return new Vec2(-a.x, -a.y)
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
export function Lerp(a, b, t) {
    return new Vec2((1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y)
}

/// Component-wise multiplication
export function Mul(a, b) {
    return new Vec2(a.x * b.x, a.y * b.y)
}

/// Multiply a scalar and vector
export function MulSV(s, v) {
    return new Vec2(s * v.x, s * v.y)
}

export function MulVS(v, s) {
    return new Vec2(v.x * s, v.y * s)
}

/// a + s * b
export function MulAdd(a, s, b) {
    return new Vec2(a.x + s * b.x, a.y + s * b.y)
}

/// a - s * b
export function MulSub(a, s, b) {
    return new Vec2(a.x - s * b.x, a.y - s * b.y)
}

/// Component-wise absolute vector
export function Abs(a) {
    return new Vec2(Math.abs(a.x), Math.abs(a.y))
}

/// Component-wise minimum vector
export function Min(a, b) {
    return new Vec2(Math.min(a.x, b.x), Math.min(a.y, b.y))
}

/// Component-wise maximum vector
export function Max(a, b) {
    return new Vec2(Math.max(a.x, b.x), Math.max(a.y, b.y))
}

/// Component-wise clamp vector v into the range [a, b]
export function Clamp(v, a, b) {
    let c = Vec2_zero
    c.x = ClampFloat(v.x, a.x, b.x)
    c.y = ClampFloat(v.y, a.y, b.y)
    return c
}

/// Get the length of this vector (the norm)
export function Length(v) {
    return Math.sqrt(v.x * v.x + v.y * v.y)
}

/// Get the distance between two points
export function Distance(a, b) {
    const dx = b.x - a.x
    const dy = b.y - a.y
    return Math.sqrt(dx * dx + dy * dy)
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
/// todo MSVC is not inlining this function in several places per warning 4710
export function Normalize(v) {
    const length = Math.sqrt(v.x * v.x + v.y * v.y)
    if (length < Number.EPSILON) {
        return Vec2_zero
    }

    const invLength = 1 / length
    const n = new Vec2(invLength * v.x, invLength * v.y)
    return n
}

/// Determines if the provided vector is normalized (norm(a) == 1).
export function IsNormalized(a) {
    const aa = Dot(a, a)
    return Math.abs(1 - aa) < 100 * Number.EPSILON
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
/// outputs the length.
export function GetLengthAndNormalize(length, v) {
    length = Math.sqrt(v.x * v.x + v.y * v.y)
    if (length < Number.EPSILON) {
        return [new Vec2(0, 0), 0]
    }

    const invLength = 1 / length
    const n = new Vec2(invLength * v.x, invLength * v.y)
    return [n, length]
}

/// Normalize rotation
export function NormalizeRot(q) {
    const mag = Math.sqrt(q.s * q.s + q.c * q.c)
    const invMag = mag > 0 ? 1 / mag : 0
    const qn = new Rot(q.c * invMag, q.s * invMag)
    return qn
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaAngle the angular displacement in radians
export function IntegrateRotation(q1, deltaAngle) {
    // dc/dt = -omega * sin(t)
    // ds/dt = omega * cos(t)
    // c2 = c1 - omega * h * s1
    // s2 = s1 + omega * h * c1
    const q2 = new Rot(q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c)
    const mag = Math.sqrt(q2.s * q2.s + q2.c * q2.c)
    const invMag = mag > 0 ? 1 / mag : 0
    const qn = new Rot(q2.c * invMag, q2.s * invMag)
    return qn
}

/// Get the length squared of this vector
export function LengthSquared(v) {
    return v.x * v.x + v.y * v.y
}

/// Get the distance squared between points
export function DistanceSquared(a, b) {
    const c = new Vec2(b.x - a.x, b.y - a.y)
    return c.x * c.x + c.y * c.y
}

/// Make a rotation using an angle in radians
export function MakeRot(radians) {
    const cs = ComputeCosSin(radians)
    return new Rot(cs.cosine, cs.sine)
}

/// Make a rotation using a unit vector
export function MakeRotFromUnitVector(unitVector) {
    console.assert(IsNormalized(unitVector))
    return new Rot(unitVector.x, unitVector.y)
}

/// Compute the rotation between two unit vectors
export function ComputeRotationBetweenUnitVectors(v1, v2) {
    console.assert(Math.abs(1 - Length(v1)) < 100 * Number.EPSILON)
    console.assert(Math.abs(1 - Length(v2)) < 100 * Number.EPSILON)

    let rot = Rot_identity
    rot.c = Dot(v1, v2)
    rot.s = Cross(v1, v2)
    return NormalizeRot(rot)
}

/// Is this rotation normalized?
export function IsNormalizedRot(q) {
    // larger tolerance due to failure on mingw 32-bit
    const qq = q.s * q.s + q.c * q.c
    return 1 - 0.0006 < qq && qq < 1 + 0.0006
}

/// Normalized linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
///	https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
export function NLerp(q1, q2, t) {
    const omt = 1 - t
    const q = new Rot(omt * q1.c + t * q2.c, omt * q1.s + t * q2.s)

    const mag = Math.sqrt(q.s * q.s + q.c * q.c)
    const invMag = mag > 0 ? 1 / mag : 0
    const qn = new Rot(q.c * invMag, q.s * invMag)
    return qn
}

/// Compute the angular velocity necessary to rotate between two rotations over a give time
/// @param q1 initial rotation
/// @param q2 final rotation
/// @param inv_h inverse time step
export function ComputeAngularVelocity(q1, q2, inv_h) {
    // ds/dt = omega * cos(t)
    // dc/dt = -omega * sin(t)
    // s2 = s1 + omega * h * c1
    // c2 = c1 - omega * h * s1

    // omega * h * s1 = c1 - c2
    // omega * h * c1 = s2 - s1
    // omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
    // omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
    // omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
    const omega = inv_h * (q2.s * q1.c - q2.c * q1.s)
    return omega
}

/// Get the angle in radians in the range [-pi, pi]
export function Rot_GetAngle(q) {
    return Atan2(q.s, q.c)
}

/// Get the x-axis
export function Rot_GetXAxis(q) {
    return new Vec2(q.c, q.s)
}

/// Get the y-axis
export function Rot_GetYAxis(q) {
    return new Vec2(-q.s, q.c)
}

/// Multiply two rotations: q * r
export function MulRot(q, r) {
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s(q + r) = qs * rc + qc * rs
    // c(q + r) = qc * rc - qs * rs
    let qr = Rot_identity
    qr.s = q.s * r.c + q.c * r.s
    qr.c = q.c * r.c - q.s * r.s
    return qr
}

/// Transpose multiply two rotations: inv(a) * b
/// This rotates a vector local in frame b into a vector local in frame a
export function InvMulRot(a, b) {
    // [ ac as] * [bc -bs] = [ac*bc+qs*bs -ac*bs+as*bc]
    // [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
    // s(a - b) = ac * bs - as * bc
    // c(a - b) = ac * bc + as * bs
    let r = Rot_identity
    r.s = a.c * b.s - a.s * b.c
    r.c = a.c * b.c + a.s * b.s
    return r
}

/// Relative angle between a and b
export function RelativeAngle(a, b) {
    // sin(b - a) = bs * ac - bc * as
    // cos(b - a) = bc * ac + bs * as
    const s = a.c * b.s - a.s * b.c
    const c = a.c * b.c + a.s * b.s
    return Atan2(s, c)
}

/// Convert any angle into the range [-pi, pi]
export function UnwindAngle(radians) {
    // Assuming this is deterministic
    return remainderf(radians, 2 * PI)
}

export function remainderf(x, y) {
    if (y === 0) {
        return NaN
    }

    const quotient = x / y

    const roundedQuotient = Math.round(quotient)
    if (
        Math.abs(quotient - roundedQuotient) === 0.5 &&
        roundedQuotient % 2 !== 0
    ) {
        const roundedQuotientToEven = Math.floor(quotient)
        const quotientDifference = quotient - roundedQuotientToEven
        const isExactlyHalfwayToEven = quotientDifference === 0.5
        const evenQuotient = roundedQuotientToEven
        const oddQuotient = roundedQuotientToEven + 1
        const finalQuotient = isExactlyHalfwayToEven ? evenQuotient : oddQuotient
        return x - finalQuotient * y
    }

    return x - roundedQuotient * y
}

/// Rotate a vector
export function RotateVector(q, v) {
    return new Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y)
}

/// Inverse rotate a vector
export function InvRotateVector(q, v) {
    return new Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y)
}

/// Transform a point (e.g. local space to world space)
export function TransformPoint(t, p) {
    const x = t.q.c * p.x - t.q.s * p.y + t.p.x
    const y = t.q.s * p.x + t.q.c * p.y + t.p.y

    return new Vec2(x, y)
}

/// Inverse transform a point (e.g. world space to local space)
export function InvTransformPoint(t, p) {
    const vx = p.x - t.p.x
    const vy = p.y - t.p.y
    return new Vec2(t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy)
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame.
/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
export function MulTransforms(A, B) {
    let C = Transform_identity
    C.q = MulRot(A.q, B.q)
    C.p = Add(RotateVector(A.q, B.p), A.p)
    return C
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
export function InvMulTransforms(A, B) {
    let C = Transform_identity
    C.q = InvMulRot(A.q, B.q)
    C.p = InvRotateVector(A.q, Sub(B.p, A.p))
    return C
}

/// Multiply a 2-by-2 matrix times a 2D vector
export function MulMV(A, v) {
    const u = new Vec2(A.cx.x * v.x + A.cy.x * v.y, A.cx.y * v.x + A.cy.y * v.y)
    return u
}

/// Get the inverse of a 2-by-2 matrix
export function GetInverse22(A) {
    const a = A.cx.x,
        b = A.cy.x,
        c = A.cx.y,
        d = A.cy.y
    let det = a * d - b * c
    if (det != 0) {
        det = 1 / det
    }

    const B = new Mat22(
        new Vec2(det * d, -det * c),
        new Vec2(-det * b, det * a)
    )
    return B
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
export function Solve22(A, b) {
    const a11 = A.cx.x,
        a12 = A.cy.x,
        a21 = A.cx.y,
        a22 = A.cy.y
    let det = a11 * a22 - a12 * a21
    if (det != 0) {
        det = 1 / det
    }
    const x = new Vec2(
        det * (a22 * b.x - a12 * b.y),
        det * (a11 * b.y - a21 * b.x)
    )
    return x
}

/// Does a fully contain b
export function AABB_Contains(a, b) {
    let s = true
    s = s && a.lowerBound.x <= b.lowerBound.x
    s = s && a.lowerBound.y <= b.lowerBound.y
    s = s && b.upperBound.x <= a.upperBound.x
    s = s && b.upperBound.y <= a.upperBound.y
    return s
}

/// Get the center of the AABB.
export function AABB_Center(a) {
    const b = new Vec2(
        0.5 * (a.lowerBound.x + a.upperBound.x),
        0.5 * (a.lowerBound.y + a.upperBound.y)
    )
    return b
}

/// Get the extents of the AABB (half-widths).
export function AABB_Extents(a) {
    const b = new Vec2(
        0.5 * (a.upperBound.x - a.lowerBound.x),
        0.5 * (a.upperBound.y - a.lowerBound.y)
    )
    return b
}

/// Union of two AABBs
export function AABB_Union(a, b) {
    let c = new AABB(Vec2_zero, Vec2_zero)
    c.lowerBound.x = Math.min(a.lowerBound.x, b.lowerBound.x)
    c.lowerBound.y = Math.min(a.lowerBound.y, b.lowerBound.y)
    c.upperBound.x = Math.max(a.upperBound.x, b.upperBound.x)
    c.upperBound.y = Math.max(a.upperBound.y, b.upperBound.y)
    return c
}

/// Do a and b overlap
export function AABB_Overlaps(a, b) {
    return !(
        b.lowerBound.x > a.upperBound.x ||
        b.lowerBound.y > a.upperBound.y ||
        a.lowerBound.x > b.upperBound.x ||
        a.lowerBound.y > b.upperBound.y
    )
}

/// Compute the bounding box of an array of circles
export function MakeAABB(points, count, radius) {
    console.assert(count > 0)
    let a = new AABB(points[0], points[0])
    for (let i = 1; i < count; ++i) {
        a.lowerBound = Min(a.lowerBound, points[i])
        a.upperBound = Max(a.upperBound, points[i])
    }

    const r = new Vec2(radius, radius)
    a.lowerBound = Sub(a.lowerBound, r)
    a.upperBound = Add(a.upperBound, r)

    return a
}

/// Signed separation of a point from a plane
export function PlaneSeparation(plane, point) {
    return Dot(plane.normal, point) - plane.offset
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += timeStep * newVelocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
export function SpringDamper(
    hertz,
    dampingRatio,
    position,
    velocity,
    timeStep
) {
    const omega = 2 * PI * hertz
    const omegaH = omega * timeStep
    return (
        (velocity - omega * omegaH * position) /
        (1 + 2 * dampingRatio * omegaH + omegaH * omegaH)
    )
}
//#endregion Math
//#region Math Overloads
export function Equals(a, b) {
    return a.x == b.x && a.y == b.y
}

export function NotEquals(a, b) {
    return a.x != b.x || a.y != b.y
}
//#endregion Math Overloads
