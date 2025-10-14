import { AABB, Plane, Rot, Vec2 } from "./math_functions";

/**
 * Low level ray cast input data
 */
export class RayCastInput {
    /**
     * 
     * @param {Vec2} origin Start point of the ray cast
     * @param {Vec2} translation Translation of the ray cast
     * @param {number} maxFraction The maximum fraction of the translation to consider, typically 1
     */
    constructor(origin, translation, maxFraction) {
        this.origin = origin;
        this.translation = translation;
        this.maxFraction = maxFraction;
    }
}

/**
 * A distance proxy is used by the GJK algorithm. It encapsulates any shape.
 * You can provide between 1 and TBD and a radius
 */
export class ShapeProxy
{
    /**
     * 
     * @param {Vec2[]} points The point cloud
     * @param {number} count The number of points. Must be greater than 0.
     * @param {number} radius The external radius of the point cloud. May be zero.
     */
    constructor(points, count, radius) {
        this.points = points;
        this.count = count;
        this.radius = radius;
    }
}

/**
 * Low level shape cast input in generic form. This allows casting an arbitrary point
 * cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
 * A capsule is two points with a non-zero radius. A box is four points with a zero radius.
 */
export class ShapeCastInput
{
    /**
     * 
     * @param {ShapeProxy} proxy A generic shape
     * @param {Vec2} translation The translation of the shape cast
     * @param {number} maxFraction The maxium fraction of the translation to consider, typically 1
     * @param {boolean} canEncroach Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
     */
    constructor(proxy, translation, maxFraction, canEncroach) {
        this.proxy = proxy;
        this.translation = translation;
        this.maxFraction = maxFraction;
        this.canEncroach = canEncroach;
    }
}

/**
 * Low level ray cast or shape-cast output data. Returns a zero fraction and normal in the case of initial overlap.
 */ 
export class CastOutput
{
	/**
	 * 
	 * @param {Vec2} normal The surface normal at the hit point
	 * @param {Vec2} point The surface hit point
	 * @param {number} fraction The fraction of the input translation at collision
	 * @param {number} iterations The number of iterations used
	 * @param {boolean} hit Did the cast hit?
	 */
	constructor(normal, point, fraction, iterations, hit) {
		this.normal = normal;
		this.point = point;
		this.fraction = fraction;
		this.iterations = iterations;
		this.hit = hit;
	}
}

/**
 * This holds the mass data computed for a shape.
 */
export class MassData
{
	/**
	 * 
	 * @param {number} mass The mass of the shape, usually in kilograms.
	 * @param {Vec2} center The position of the shape's centroid relative to the shape's origin.
	 * @param {number} rotationalInertia The rotational inertia of the shape about the shape center.
	 */
	constructor(mass, center, rotationalInertia) {
		this.mass = mass;
		this.center = center;
		this.rotationalInertia = rotationalInertia;
	}
}

/**
 * A solid circle
 */
export class Circle
{
	/**
	 * 
	 * @param {Vec2} center The local center
	 * @param {number} radius The radius
	 */
	constructor(center, radius) {
		this.center = center;
		this.radius = radius;
	}
}

/**
 * A solid capsule can be viewed as two semicircles connected
 * by a rectangle.
 */
export class Capsule
{
	/**
	 * 
	 * @param {Vec2} center1 Local center of the first semicircle
	 * @param {Vec2} center2 Local center of the second semicircle
	 * @param {number} radius The radius of the semicircles
	 */
	constructor(center1, center2, radius) {
		this.center1 = center1;
		this.center2 = center2;
		this.radius = radius;
	}
}

/**
 * A solid convex polygon. It is assumed that the interior of the polygon is to
 * the left of each edge.
 * Polygons have a maximum number of vertices equal to MAX_POLYGON_VERTICES.
 * In most cases you should not need many vertices for a convex polygon.
 * @warning DO NOT fill this out manually, instead use a helper function like MakePolygon or MakeBox.
 */
export class Polygon
{
	/**
	 * 
	 * @param {Vec2} vertices The polygon vertices
	 * @param {Vec2} normals The outward normal vectors of the polygon sides
	 * @param {Vec2} centroid The centroid of the polygon
	 * @param {number} radius The external radius for rounded polygons
	 * @param {number} count The number of polygon vertices
	 */
	constructor(vertices, normals, centroid, radius, count) {
		this.vertices = vertices;
		this.normals = normals;
		this.centroid = centroid;
		this.radius = radius;
		this.count = count;
	}
}

/**
 * A line segment with two-sided collision.
 */
export class Segment
{
	/**
	 * 
	 * @param {Vec2} point1 The first point
	 * @param {Vec2} point2 The second point
	 */
	constructor(point1, point2) {
		this.point1 = point1;
		this.point2 = point2;
	}
}

/**
 * A line segment with one-sided collision. Only collides on the right side.
 * Several of these are generated for a chain shape.
 * ghost1 -> point1 -> point2 -> ghost2
 */
export class ChainSegment
{
	/**
	 * 
	 * @param {Vec2} ghost1 The tail ghost vertex
	 * @param {Segment} segment The line segment
	 * @param {Vec2} ghost2 The head ghost vertex
	 * @param {number} chainId The owning chain shape index (internal usage only)
	 */
	constructor(ghost1, segment, ghost2, chainId) {
		this.ghost1 = ghost1;
		this.segment = segment;
		this.ghost2 = ghost2;
		this.chainId = chainId;
	}
}

/*
/// Validate ray cast input data (NaN, etc)
API bool IsValidRay( const RayCastInput* input );

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from ComputeHull
API Polygon MakePolygon( const Hull* hull, float radius );

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from ComputeHull
API Polygon MakeOffsetPolygon( const Hull* hull, Vec2 position, Rot rotation );

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from ComputeHull
API Polygon MakeOffsetRoundedPolygon( const Hull* hull, Vec2 position, Rot rotation, float radius );

/// Make a square polygon, bypassing the need for a convex hull.
/// @param halfWidth the half-width
API Polygon MakeSquare( float halfWidth );

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
API Polygon MakeBox( float halfWidth, float halfHeight );

/// Make a rounded box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param radius the radius of the rounded extension
API Polygon MakeRoundedBox( float halfWidth, float halfHeight, float radius );

/// Make an offset box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param center the local center of the box
/// @param rotation the local rotation of the box
API Polygon MakeOffsetBox( float halfWidth, float halfHeight, Vec2 center, Rot rotation );

/// Make an offset rounded box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param center the local center of the box
/// @param rotation the local rotation of the box
/// @param radius the radius of the rounded extension
API Polygon MakeOffsetRoundedBox( float halfWidth, float halfHeight, Vec2 center, Rot rotation, float radius );

/// Transform a polygon. This is useful for transferring a shape from one body to another.
API Polygon TransformPolygon( Transform transform, const Polygon* polygon );

/// Compute mass properties of a circle
API MassData ComputeCircleMass( const Circle* shape, float density );

/// Compute mass properties of a capsule
API MassData ComputeCapsuleMass( const Capsule* shape, float density );

/// Compute mass properties of a polygon
API MassData ComputePolygonMass( const Polygon* shape, float density );

/// Compute the bounding box of a transformed circle
API AABB ComputeCircleAABB( const Circle* shape, Transform transform );

/// Compute the bounding box of a transformed capsule
API AABB ComputeCapsuleAABB( const Capsule* shape, Transform transform );

/// Compute the bounding box of a transformed polygon
API AABB ComputePolygonAABB( const Polygon* shape, Transform transform );

/// Compute the bounding box of a transformed line segment
API AABB ComputeSegmentAABB( const Segment* shape, Transform transform );

/// Test a point for overlap with a circle in local space
API bool PointInCircle( const Circle* shape, Vec2 point );

/// Test a point for overlap with a capsule in local space
API bool PointInCapsule( const Capsule* shape, Vec2 point );

/// Test a point for overlap with a convex polygon in local space
API bool PointInPolygon( const Polygon* shape, Vec2 point );

/// Ray cast versus circle shape in local space.
API CastOutput RayCastCircle( const Circle* shape, const RayCastInput* input );

/// Ray cast versus capsule shape in local space.
API CastOutput RayCastCapsule( const Capsule* shape, const RayCastInput* input );

/// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
API CastOutput RayCastSegment( const Segment* shape, const RayCastInput* input, bool oneSided );

/// Ray cast versus polygon shape in local space.
API CastOutput RayCastPolygon( const Polygon* shape, const RayCastInput* input );

/// Shape cast versus a circle.
API CastOutput ShapeCastCircle(const Circle* shape,  const ShapeCastInput* input );

/// Shape cast versus a capsule.
API CastOutput ShapeCastCapsule( const Capsule* shape, const ShapeCastInput* input);

/// Shape cast versus a line segment.
API CastOutput ShapeCastSegment( const Segment* shape, const ShapeCastInput* input );

/// Shape cast versus a convex polygon.
API CastOutput ShapeCastPolygon( const Polygon* shape, const ShapeCastInput* input );
*/

/**
 * A convex hull. Used to create convex polygons.
 * @warning Do not modify these values directly, instead use ComputeHull()
 */
export class Hull
{
	/**
	 * 
	 * @param {Vec2} points The final points of the hull
	 * @param {number} count The number of points
	 */
	constructor(points, count) {
		this.points = points;
		this.count = count;
	}
}

/*
/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - less than 3 points
/// - more than MAX_POLYGON_VERTICES points
/// This welds close points and removes collinear points.
/// @warning Do not modify a hull once it has been computed
API Hull ComputeHull( const Vec2* points, int count );

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
API bool ValidateHull( const Hull* hull );
*/

//#region Distance
/*
Functions for computing the distance between shapes.

These are advanced functions you can use to perform distance calculations. There
are functions for computing the closest points between shapes, doing linear shape casts,
and doing rotational shape casts. The latter is called time of impact (TOI).
*/

/**
 * Result of computing the distance between two line segments
 */
export class SegmentDistanceResult
{
	/**
	 * 
	 * @param {Vec2} closest1 The closest point on the first segment
	 * @param {Vec2} closest2 The closest point on the second segment
	 * @param {number} fraction1 The barycentric coordinate on the first segment
	 * @param {number} fraction2 The barycentric coordinate on the second segment
	 * @param {number} distanceSquared The squared distance between the closest points
	 */
	constructor(closest1, closest2, fraction1, fraction2, distanceSquared) {
		this.closest1 = closest1;
		this.closest2 = closest2;
		this.fraction1 = fraction1;
		this.fraction2 = fraction2;
		this.distanceSquared = distanceSquared;
	}
}

/*
/// Compute the distance between two line segments, clamping at the end points if needed.
API SegmentDistanceResult SegmentDistance( Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2 );
*/

/**
 * Used to warm start the GJK simplex. If you call this function multiple times with nearby
 * transforms this might improve performance. Otherwise you can zero initialize this.
 * The distance cache must be initialized to zero on the first call.
 * Users should generally just zero initialize this structure for each call.
 */
export class SimplexCache
{
	/**
	 * 
	 * @param {number} count The number of stored simplex points
	 * @param {number[]} indexA [3] The cached simplex indices on shape A
	 * @param {number[]} indexB [3] The cached simplex indices on shape B
	 */
	constructor(count, indexA, indexB) {
		this.count = count;
		this.indexA = indexA;
		this.indexB = indexB;
	}
}

// static const SimplexCache emptySimplexCache = ZERO_INIT;

/**
 * Input for ShapeDistance
 */
export class DistanceInput
{
	/**
	 * 
	 * @param {ShapeProxy} proxyA The proxy for shape A
	 * @param {ShapeProxy} proxyB The proxy for shape B
	 * @param {Transform} transformA The world transform for shape A
	 * @param {Transform} transformB The world transform for shape B
	 * @param {boolean} useRadii Should the proxy radius be considered?
	 */
	constructor(proxyA, proxyB, transformA, transformB, useRadii) {
		this.proxyA = proxyA;
		this.proxyB = proxyB;
		this.transformA = transformA;
		this.transformB = transformB;
		this.useRadii = useRadii;
	}
}

/**
 * Output for ShapeDistance
 */
export class DistanceOutput
{
	/**
	 * 
	 * @param {Vec2} pointA Closest point on shapeA
	 * @param {Vec2} pointB Closest point on shapeB
	 * @param {Vec2} normal Normal vector that points from A to B. Invalid if distance is zero.
	 * @param {number} distance The final distance, zero if overlapped
	 * @param {number} iterations Number of GJK iterations used
	 * @param {number} simplexCount The number of simplexes stored in the simplex array
	 */
	constructor(pointA, pointB, normal, distance, iterations, simplexCount) {
		this.pointA = pointA;
		this.pointB = pointB;
		this.normal = normal;
		this.distance = distance;
		this.iterations = iterations;
		this.simplexCount = simplexCount;
	}
}

/**
 * Simplex vertex for debugging the GJK algorithm
 */
export class SimplexVertex
{
	/**
	 * 
	 * @param {Vec2} wA Support point in proxyA
	 * @param {Vec2} wB Support point in proxyB
	 * @param {Vec2} w wB - wA
	 * @param {number} a Barycentric coordinate for closest point
	 * @param {number} indexA wA index
	 * @param {number} indexB wB index
	 */
	constructor(wA, wB, w, a, indexA, indexB) {
		this.wA = wA;
		this.wB = wB;
		this.w = w;
		this.a = a;
		this.indexA = indexA;
		this.indexB = indexB;
	}
}

/**
 * Simplex from the GJK algorithm
 */
export class Simplex
{
	/**
	 * 
	 * @param {SimplexVertex} v1 Vertices
	 * @param {SimplexVertex} v2 Vertices
	 * @param {SimplexVertex} v3 Vertices
	 * @param {number} count Number of valid vertices
	 */
	constructor(v1, v2, v3, count) {
		this.v1 = v1;
		this.v2 = v2;
		this.v3 = v3;
		this.count = count;
	}
}

/*
/// Compute the closest points between two shapes represented as point clouds.
/// SimplexCache cache is input/output. On the first call set SimplexCache.count to zero.
/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
API DistanceOutput ShapeDistance( const DistanceInput* input, SimplexCache* cache, Simplex* simplexes,
										 int simplexCapacity );
*/

/**
 * Input parameters for ShapeCast
 */
export class ShapeCastPairInput
{
	/**
	 * 
	 * @param {ShapeProxy} proxyA The proxy for shape A
	 * @param {ShapeProxy} proxyB The proxy for shape B
	 * @param {Transform} transformA The world transform for shape A
	 * @param {Transform} transformB The world transform for shape B
	 * @param {Vec2} translationB The translation of shape B
	 * @param {number} maxFraction The fraction of the translation to consider, typically 1
	 * @param {boolean} canEncroach Allows shapes with a radius to move slightly closer if already touching
	 */
	constructor(proxyA, proxyB, transformA, transformB, translationB, maxFraction, canEncroach) {
		this.proxyA = proxyA;
		this.proxyB = proxyB;
		this.transformA = transformA;
		this.transformB = transformB;
		this.translationB = translationB;
		this.maxFraction = maxFraction;
		this.canEncroach = canEncroach;
	}
}

/*
/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
/// Initially touching shapes are treated as a miss.
API CastOutput ShapeCast( const ShapeCastPairInput* input );

/// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.
API ShapeProxy MakeProxy( const Vec2* points, int count, float radius );

/// Make a proxy with a transform. This is a deep copy of the points.
API ShapeProxy MakeOffsetProxy( const Vec2* points, int count, float radius, Vec2 position, Rot rotation );
*/

/**
 * This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
 * which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
 * position.
 */
export class Sweep
{
	/**
	 * 
	 * @param {Vec2} localCenter Locale center of mass position
	 * @param {Vec2} c1 Starting center of mass world position
	 * @param {Vec2} c2 Ending center of mass world position
	 * @param {Rot} q1 Starting world rotation
	 * @param {Rot} q2 Ending world rotation
	 */
	constructor(localCenter, c1, c2, q1, q2) {
		this.localCenter = localCenter;
		this.c1 = c1;
		this.c2 = c2;
		this.q1 = q1;
		this.q2 = q2;
	}
}

/*
/// Evaluate the transform sweep at a specific time.
API Transform GetSweepTransform( const Sweep* sweep, float time );
*/

/**
 * Time of impact input
 */
export class TOIInput
{
	/**
	 * 
	 * @param {ShapeProxy} proxyA The proxy for shape A
	 * @param {ShapeProxy} proxyB The proxy for shape B
	 * @param {Sweep} sweepA The movement of shape A
	 * @param {Sweep} sweepB The movement of shape B
	 * @param {number} maxFraction Defines the sweep interval [0, maxFraction]
	 */
	constructor(proxyA, proxyB, sweepA, sweepB, maxFraction) {
		this.proxyA = proxyA;
		this.proxyB = proxyB;
		this.sweepA = sweepA;
		this.sweepB = sweepB;
		this.maxFraction = maxFraction;
	}
}

/**
 * Describes the TOI output
 */
export const TOIState = {
	toiStateUnknown: 0,
	toiStateFailed: 1,
	toiStateOverlapped: 2,
	toiStateHit: 3,
	toiStateSeparated: 4
}

/**
 * Time of impact output
 */
export class TOIOutput
{
	/**
	 * 
	 * @param {TOIState} state The type of result
	 * @param {Vec2} point The hit point
	 * @param {Vec2} normal The hit normal
	 * @param {number} fraction The sweep time of the collision
	 */
	constructor(state, point, normal, fraction) {
		this.state = state;
		this.point = point;
		this.normal = normal;
		this.fraction = fraction;
	}
}

/*
/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
API TOIOutput TimeOfImpact( const TOIInput* input );
*/

//#endregion Distance

//#region Collision

/**
 * A manifold point is a contact point belonging to a contact manifold.
 * It holds details related to the geometry and dynamics of the contact points.
 * Box2D uses speculative collision so some contact points may be separated.
 * You may use the totalNormalImpulse to determine if there was an interaction during
 * the time step.
 */
export class ManifoldPoint
{
	/**
	 * 
	 * @param {Vec2} point Location of the contact point in world space. Subject to precision loss at large coordinates.(Should only be used for debugging.)
	 * @param {Vec2} anchorA Location of the contact point relative to shapeA's origin in world space(When used internally to the Box2D solver, this is relative to the body center of mass.)
	 * @param {Vec2} anchorB Location of the contact point relative to shapeB's origin in world space(When used internally to the Box2D solver, this is relative to the body center of mass.)
	 * @param {number} separation The separation of the contact point, negative if penetrating
	 * @param {number} normalImpulse The impulse along the manifold normal vector.
	 * @param {number} tangentImpulse The friction impulse
	 * @param {number} totalNormalImpulse The total normal impulse applied across sub-stepping and restitution. This is important to identify speculative contact points that had an interaction in the time step.
	 * @param {number} normalVelocity Relative normal velocity pre-solve. Used for hit events. If the normal impulse is zero then there was no hit. Negative means shapes are approaching.
	 * @param {number} id Uniquely identifies a contact point between two shapes
	 * @param {boolean} persisted Did this contact point exist the previous step?
	 */
	constructor(point, anchorA, anchorB, separation, normalImpulse, tangentImpulse, totalNormalImpulse, normalVelocity, id, persisted) {
		this.point = point;
		this.anchorA = anchorA;
		this.anchorB = anchorB;
		this.separation = separation;
		this.normalImpulse = normalImpulse;
		this.tangentImpulse = tangentImpulse;
		this.totalNormalImpulse = totalNormalImpulse;
		this.normalVelocity = normalVelocity;
		this.id = id;
		this.persisted = persisted;
	}
}

/**
 * A contact manifold describes the contact points between colliding shapes.
 * @note Box2D uses speculative collision so some contact points may be separated.
 */
export class Manifold
{
	/**
	 * 
	 * @param {Vec2} normal The unit normal vector in world space, points from shape A to bodyB
	 * @param {number} rollingImpulse Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	 * @param {ManifoldPoint} points [2]The manifold points, up to two are possible in 2D
	 * @param {number} pointCount The number of contacts points, will be 0, 1, or 2
	 */
	constructor(normal, rollingImpulse, points, pointCount) {
		this.normal = normal;
		this.rollingImpulse = rollingImpulse;
		this.points = points;
		this.pointCount = pointCount;
	}
}

/*
/// Compute the contact manifold between two circles
API Manifold CollideCircles( const Circle* circleA, Transform xfA, const Circle* circleB, Transform xfB );

/// Compute the contact manifold between a capsule and circle
API Manifold CollideCapsuleAndCircle( const Capsule* capsuleA, Transform xfA, const Circle* circleB,
											 Transform xfB );

/// Compute the contact manifold between an segment and a circle
API Manifold CollideSegmentAndCircle( const Segment* segmentA, Transform xfA, const Circle* circleB,
											 Transform xfB );

/// Compute the contact manifold between a polygon and a circle
API Manifold CollidePolygonAndCircle( const Polygon* polygonA, Transform xfA, const Circle* circleB,
											 Transform xfB );

/// Compute the contact manifold between a capsule and circle
API Manifold CollideCapsules( const Capsule* capsuleA, Transform xfA, const Capsule* capsuleB, Transform xfB );

/// Compute the contact manifold between an segment and a capsule
API Manifold CollideSegmentAndCapsule( const Segment* segmentA, Transform xfA, const Capsule* capsuleB,
											  Transform xfB );

/// Compute the contact manifold between a polygon and capsule
API Manifold CollidePolygonAndCapsule( const Polygon* polygonA, Transform xfA, const Capsule* capsuleB,
											  Transform xfB );

/// Compute the contact manifold between two polygons
API Manifold CollidePolygons( const Polygon* polygonA, Transform xfA, const Polygon* polygonB, Transform xfB );

/// Compute the contact manifold between an segment and a polygon
API Manifold CollideSegmentAndPolygon( const Segment* segmentA, Transform xfA, const Polygon* polygonB,
											  Transform xfB );

/// Compute the contact manifold between a chain segment and a circle
API Manifold CollideChainSegmentAndCircle( const ChainSegment* segmentA, Transform xfA, const Circle* circleB,
												  Transform xfB );

/// Compute the contact manifold between a chain segment and a capsule
API Manifold CollideChainSegmentAndCapsule( const ChainSegment* segmentA, Transform xfA, const Capsule* capsuleB,
												   Transform xfB, SimplexCache* cache );

/// Compute the contact manifold between a chain segment and a rounded polygon
API Manifold CollideChainSegmentAndPolygon( const ChainSegment* segmentA, Transform xfA, const Polygon* polygonB,
												   Transform xfB, SimplexCache* cache );
*/
//#endregion Collision

//#region Dynamic Tree

/*
The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
This data structure may have uses in games for organizing other geometry data and may be used independently
of Box2D rigid body simulation.
A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
A dynamic tree arranges data in a binary tree to accelerate
queries such as AABB queries and ray casts. Leaf nodes are proxies
with an AABB. These are used to hold a user collision object.
Nodes are pooled and relocatable, so I use node indices rather than pointers.
The dynamic tree is made available for advanced users that would like to use it to organize
spatial game data besides rigid bodies.
*/

/**
 * The dynamic tree structure. This should be considered private data.
 * It is placed here for performance reasons.
 */
export class DynamicTree
{
	/**
	 * 
	 * @param {TreeNode} nodes The tree nodes
	 * @param {number} root The root index
	 * @param {number} nodeCount The number of nodes
	 * @param {number} nodeCapacity The allocated node space
	 * @param {number} freeList Node free list
	 * @param {number} proxyCount Number of proxies created
	 * @param {number} leafIndices Leaf indices for rebuild
	 * @param {AABB} leafBoxes Leaf bounding boxes for rebuild
	 * @param {Vec2} leafCenters Leaf bounding box centers for rebuild
	 * @param {number} binIndices Bins for sorting during rebuild
	 * @param {number} rebuildCapacity Allocated space for rebuilding
	 */
	constructor(nodes, root, nodeCount, nodeCapacity, freeList, proxyCount, leafIndices, leafBoxes, leafCenters, binIndices, rebuildCapacity) {
		this.nodes = nodes;
		this.root = root;
		this.nodeCount = nodeCount;
		this.nodeCapacity = nodeCapacity;
		this.freeList = freeList;
		this.proxyCount = proxyCount;
		this.leafIndices = leafIndices;
		this.leafBoxes = leafBoxes;
		this.leafCenters = leafCenters;
		this.binIndices = binIndices;
		this.rebuildCapacity = rebuildCapacity;
	}
}

/**
 * These are performance results returned by dynamic tree queries.
 */
export class TreeStats
{
	/**
	 * 
	 * @param {number} nodeVisits Number of internal nodes visited during the query
	 * @param {number} leafVisits Number of leaf nodes visited during the query
	 */
	constructor(nodeVisits, leafVisits) {
		this.nodeVisits = nodeVisits;
		this.leafVisits = leafVisits;
	}
}

/*
/// Constructing the tree initializes the node pool.
API DynamicTree DynamicTree_Create( void );

/// Destroy the tree, freeing the node pool.
API void DynamicTree_Destroy( DynamicTree* tree );

/// Create a proxy. Provide an AABB and a userData value.
API int DynamicTree_CreateProxy( DynamicTree* tree, AABB aabb, uint64_t categoryBits, uint64_t userData );

/// Destroy a proxy. This asserts if the id is invalid.
API void DynamicTree_DestroyProxy( DynamicTree* tree, int proxyId );

/// Move a proxy to a new AABB by removing and reinserting into the tree.
API void DynamicTree_MoveProxy( DynamicTree* tree, int proxyId, AABB aabb );

/// Enlarge a proxy and enlarge ancestors as necessary.
API void DynamicTree_EnlargeProxy( DynamicTree* tree, int proxyId, AABB aabb );

/// Modify the category bits on a proxy. This is an expensive operation.
API void DynamicTree_SetCategoryBits( DynamicTree* tree, int proxyId, uint64_t categoryBits );

/// Get the category bits on a proxy.
API uint64_t DynamicTree_GetCategoryBits( DynamicTree* tree, int proxyId );

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
typedef bool TreeQueryCallbackFcn( int proxyId, uint64_t userData, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
///	@return performance data
API TreeStats DynamicTree_Query( const DynamicTree* tree, AABB aabb, uint64_t maskBits,
										TreeQueryCallbackFcn* callback, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
/// No filtering is performed.
///	@return performance data
API TreeStats DynamicTree_QueryAll( const DynamicTree* tree, AABB aabb, TreeQueryCallbackFcn* callback,
										   void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float TreeRayCastCallbackFcn( const RayCastInput* input, int proxyId, uint64_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
///	However, this filtering may be approximate, so the user should still apply filtering to results.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
/// @param maskBits mask bit hint: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the ray
/// @param context user context that is passed to the callback
///	@return performance data
API TreeStats DynamicTree_RayCast( const DynamicTree* tree, const RayCastInput* input, uint64_t maskBits,
										  TreeRayCastCallbackFcn* callback, void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float TreeShapeCastCallbackFcn( const ShapeCastInput* input, int proxyId, uint64_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the shape
/// @param context user context that is passed to the callback
///	@return performance data
API TreeStats DynamicTree_ShapeCast( const DynamicTree* tree, const ShapeCastInput* input, uint64_t maskBits,
											TreeShapeCastCallbackFcn* callback, void* context );

/// Get the height of the binary tree.
API int DynamicTree_GetHeight( const DynamicTree* tree );

/// Get the ratio of the sum of the node areas to the root area.
API float DynamicTree_GetAreaRatio( const DynamicTree* tree );

/// Get the bounding box that contains the entire tree
API AABB DynamicTree_GetRootBounds( const DynamicTree* tree );

/// Get the number of proxies created
API int DynamicTree_GetProxyCount( const DynamicTree* tree );

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
API int DynamicTree_Rebuild( DynamicTree* tree, bool fullBuild );

/// Get the number of bytes used by this tree
API int DynamicTree_GetByteCount( const DynamicTree* tree );

/// Get proxy user data
API uint64_t DynamicTree_GetUserData( const DynamicTree* tree, int proxyId );

/// Get the AABB of a proxy
API AABB DynamicTree_GetAABB( const DynamicTree* tree, int proxyId );

/// Validate this tree. For testing.
API void DynamicTree_Validate( const DynamicTree* tree );

/// Validate this tree has no enlarged AABBs. For testing.
API void DynamicTree_ValidateNoEnlarged( const DynamicTree* tree );
*/
//#endregion Dynamic Tree

//#region Character mover

/**
 * These are the collision planes returned from World_CollideMover
 */
export class PlaneResult
{
	/**
	 * 
	 * @param {Plane} plane The collision plane between the mover and a convex shape
	 * @param {Vec2} point The collision point on the shape.
	 * @param {boolean} hit Did the collision register a hit? If not this plane should be ignored.
	 */
	constructor(plane, point, hit) {
		this.plane = plane;
		this.point = point;
		this.hit = hit;
	}
}

/**
 * These are collision planes that can be fed to SolvePlanes. Normally
 * this is assembled by the user from plane results in PlaneResult
 */
export class CollisionPlane
{
	/**
	 * 
	 * @param {Plane} plane The collision plane between the mover and some shape
	 * @param {number} pushLimit Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can make the plane collision soft. Usually in meters.
	 * @param {number} push The push on the mover determined by SolvePlanes. Usually in meters.
	 * @param {boolean} clipVelocity Indicates if ClipVector should clip against this plane. Should be false for soft collision.
	 */
	constructor(plane, pushLimit, push, clipVelocity) {
		this.plane = plane;
		this.pushLimit = pushLimit;
		this.push = push;
		this.clipVelocity = clipVelocity;
	}
}

/**
 * Result returned by SolvePlanes
 */
export class PlaneSolverResult
{
	/**
	 * 
	 * @param {Vec2} translation The translation of the mover
	 * @param {number} iterationCount The number of iterations used by the plane solver. For diagnostics.
	 */
	constructor(translation, iterationCount) {
		this.translation = translation;
		this.iterationCount = iterationCount;
	}
}

/*
/// Solves the position of a mover that satisfies the given collision planes.
/// @param targetDelta the desired movement from the position used to generate the collision planes
/// @param planes the collision planes
/// @param count the number of collision planes
API PlaneSolverResult SolvePlanes( Vec2 targetDelta, CollisionPlane* planes, int count );

/// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
/// set to false are skipped.
API Vec2 ClipVector( Vec2 vector, const CollisionPlane* planes, int count );
*/
//#endregion Character mover
