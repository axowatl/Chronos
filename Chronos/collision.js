import { Rot, Vec2 } from "./math_functions";

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
export class b2CastOutput
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
export class b2MassData
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
export class b2Circle
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
export class b2Capsule
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
 * Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
 * In most cases you should not need many vertices for a convex polygon.
 * @warning DO NOT fill this out manually, instead use a helper function like MakePolygon or MakeBox.
 */
export class b2Polygon
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
export class b2Segment
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
export class b2ChainSegment
{
	/**
	 * 
	 * @param {Vec2} ghost1 The tail ghost vertex
	 * @param {b2Segment} segment The line segment
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
B2_API bool b2IsValidRay( const b2RayCastInput* input );

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API b2Polygon b2MakePolygon( const b2Hull* hull, float radius );

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API b2Polygon b2MakeOffsetPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation );

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API b2Polygon b2MakeOffsetRoundedPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation, float radius );

/// Make a square polygon, bypassing the need for a convex hull.
/// @param halfWidth the half-width
B2_API b2Polygon b2MakeSquare( float halfWidth );

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
B2_API b2Polygon b2MakeBox( float halfWidth, float halfHeight );

/// Make a rounded box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param radius the radius of the rounded extension
B2_API b2Polygon b2MakeRoundedBox( float halfWidth, float halfHeight, float radius );

/// Make an offset box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param center the local center of the box
/// @param rotation the local rotation of the box
B2_API b2Polygon b2MakeOffsetBox( float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation );

/// Make an offset rounded box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param center the local center of the box
/// @param rotation the local rotation of the box
/// @param radius the radius of the rounded extension
B2_API b2Polygon b2MakeOffsetRoundedBox( float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation, float radius );

/// Transform a polygon. This is useful for transferring a shape from one body to another.
B2_API b2Polygon b2TransformPolygon( b2Transform transform, const b2Polygon* polygon );

/// Compute mass properties of a circle
B2_API b2MassData b2ComputeCircleMass( const b2Circle* shape, float density );

/// Compute mass properties of a capsule
B2_API b2MassData b2ComputeCapsuleMass( const b2Capsule* shape, float density );

/// Compute mass properties of a polygon
B2_API b2MassData b2ComputePolygonMass( const b2Polygon* shape, float density );

/// Compute the bounding box of a transformed circle
B2_API b2AABB b2ComputeCircleAABB( const b2Circle* shape, b2Transform transform );

/// Compute the bounding box of a transformed capsule
B2_API b2AABB b2ComputeCapsuleAABB( const b2Capsule* shape, b2Transform transform );

/// Compute the bounding box of a transformed polygon
B2_API b2AABB b2ComputePolygonAABB( const b2Polygon* shape, b2Transform transform );

/// Compute the bounding box of a transformed line segment
B2_API b2AABB b2ComputeSegmentAABB( const b2Segment* shape, b2Transform transform );

/// Test a point for overlap with a circle in local space
B2_API bool b2PointInCircle( const b2Circle* shape, b2Vec2 point );

/// Test a point for overlap with a capsule in local space
B2_API bool b2PointInCapsule( const b2Capsule* shape, b2Vec2 point );

/// Test a point for overlap with a convex polygon in local space
B2_API bool b2PointInPolygon( const b2Polygon* shape, b2Vec2 point );

/// Ray cast versus circle shape in local space.
B2_API b2CastOutput b2RayCastCircle( const b2Circle* shape, const b2RayCastInput* input );

/// Ray cast versus capsule shape in local space.
B2_API b2CastOutput b2RayCastCapsule( const b2Capsule* shape, const b2RayCastInput* input );

/// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
B2_API b2CastOutput b2RayCastSegment( const b2Segment* shape, const b2RayCastInput* input, bool oneSided );

/// Ray cast versus polygon shape in local space.
B2_API b2CastOutput b2RayCastPolygon( const b2Polygon* shape, const b2RayCastInput* input );

/// Shape cast versus a circle.
B2_API b2CastOutput b2ShapeCastCircle(const b2Circle* shape,  const b2ShapeCastInput* input );

/// Shape cast versus a capsule.
B2_API b2CastOutput b2ShapeCastCapsule( const b2Capsule* shape, const b2ShapeCastInput* input);

/// Shape cast versus a line segment.
B2_API b2CastOutput b2ShapeCastSegment( const b2Segment* shape, const b2ShapeCastInput* input );

/// Shape cast versus a convex polygon.
B2_API b2CastOutput b2ShapeCastPolygon( const b2Polygon* shape, const b2ShapeCastInput* input );
*/

/**
 * A convex hull. Used to create convex polygons.
 * @warning Do not modify these values directly, instead use ComputeHull()
 */
export class b2Hull
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
/// - more than B2_MAX_POLYGON_VERTICES points
/// This welds close points and removes collinear points.
/// @warning Do not modify a hull once it has been computed
B2_API b2Hull b2ComputeHull( const b2Vec2* points, int count );

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
B2_API bool b2ValidateHull( const b2Hull* hull );
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
export class b2SegmentDistanceResult
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
B2_API b2SegmentDistanceResult b2SegmentDistance( b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2 );
*/

/**
 * Used to warm start the GJK simplex. If you call this function multiple times with nearby
 * transforms this might improve performance. Otherwise you can zero initialize this.
 * The distance cache must be initialized to zero on the first call.
 * Users should generally just zero initialize this structure for each call.
 */
export class b2SimplexCache
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

// static const b2SimplexCache b2_emptySimplexCache = B2_ZERO_INIT;

/**
 * Input for b2ShapeDistance
 */
export class b2DistanceInput
{
	/**
	 * 
	 * @param {b2ShapeProxy} proxyA The proxy for shape A
	 * @param {b2ShapeProxy} proxyB The proxy for shape B
	 * @param {b2Transform} transformA The world transform for shape A
	 * @param {b2Transform} transformB The world transform for shape B
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
 * Output for b2ShapeDistance
 */
export class b2DistanceOutput
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
export class b2SimplexVertex
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
export class b2Simplex
{
	/**
	 * 
	 * @param {b2SimplexVertex} v1 Vertices
	 * @param {b2SimplexVertex} v2 Vertices
	 * @param {b2SimplexVertex} v3 Vertices
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
/// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
B2_API b2DistanceOutput b2ShapeDistance( const b2DistanceInput* input, b2SimplexCache* cache, b2Simplex* simplexes,
										 int simplexCapacity );
*/

/**
 * Input parameters for b2ShapeCast
 */
export class b2ShapeCastPairInput
{
	/**
	 * 
	 * @param {b2ShapeProxy} proxyA The proxy for shape A
	 * @param {b2ShapeProxy} proxyB The proxy for shape B
	 * @param {b2Transform} transformA The world transform for shape A
	 * @param {b2Transform} transformB The world transform for shape B
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
B2_API b2CastOutput b2ShapeCast( const b2ShapeCastPairInput* input );

/// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.
B2_API b2ShapeProxy b2MakeProxy( const b2Vec2* points, int count, float radius );

/// Make a proxy with a transform. This is a deep copy of the points.
B2_API b2ShapeProxy b2MakeOffsetProxy( const b2Vec2* points, int count, float radius, b2Vec2 position, b2Rot rotation );
*/

/**
 * This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
 * which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
 * position.
 */
export class b2Sweep
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
B2_API b2Transform b2GetSweepTransform( const b2Sweep* sweep, float time );
*/

/**
 * Time of impact input
 */
export class b2TOIInput
{
	/**
	 * 
	 * @param {b2ShapeProxy} proxyA The proxy for shape A
	 * @param {b2ShapeProxy} proxyB The proxy for shape B
	 * @param {b2Sweep} sweepA The movement of shape A
	 * @param {b2Sweep} sweepB The movement of shape B
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
enum b2TOIState
{
	b2_toiStateUnknown,
	b2_toiStateFailed,
	b2_toiStateOverlapped,
	b2_toiStateHit,
	b2_toiStateSeparated
}

/// Time of impact output
typedef struct b2TOIOutput
{
	/// The type of result
	b2TOIState state;

	/// The hit point
	b2Vec2 point;

	/// The hit normal
	b2Vec2 normal;

	/// The sweep time of the collision 
	float fraction;
} b2TOIOutput;

/*
/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
B2_API b2TOIOutput b2TimeOfImpact( const b2TOIInput* input );
*/

//#endregion Distance

/**
 * @defgroup collision Collision
 * @brief Functions for colliding pairs of shapes
 * @{
 */

/// A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box2D uses speculative collision so some contact points may be separated.
/// You may use the totalNormalImpulse to determine if there was an interaction during
/// the time step.
typedef struct b2ManifoldPoint
{
	/// Location of the contact point in world space. Subject to precision loss at large coordinates.
	/// @note Should only be used for debugging.
	b2Vec2 point;

	/// Location of the contact point relative to shapeA's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	b2Vec2 anchorA;

	/// Location of the contact point relative to shapeB's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	b2Vec2 anchorB;

	/// The separation of the contact point, negative if penetrating
	float separation;

	/// The impulse along the manifold normal vector.
	float normalImpulse;

	/// The friction impulse
	float tangentImpulse;

	/// The total normal impulse applied across sub-stepping and restitution. This is important
	/// to identify speculative contact points that had an interaction in the time step.
	float totalNormalImpulse;

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	float normalVelocity;

	/// Uniquely identifies a contact point between two shapes
	uint16_t id;

	/// Did this contact point exist the previous step?
	bool persisted;
} b2ManifoldPoint;

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box2D uses speculative collision so some contact points may be separated.
typedef struct b2Manifold
{
	/// The unit normal vector in world space, points from shape A to bodyB
	b2Vec2 normal;

	/// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	float rollingImpulse;

	/// The manifold points, up to two are possible in 2D
	b2ManifoldPoint points[2];

	/// The number of contacts points, will be 0, 1, or 2
	int pointCount;

} b2Manifold;

/*
/// Compute the contact manifold between two circles
B2_API b2Manifold b2CollideCircles( const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB );

/// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsuleAndCircle( const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

/// Compute the contact manifold between an segment and a circle
B2_API b2Manifold b2CollideSegmentAndCircle( const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

/// Compute the contact manifold between a polygon and a circle
B2_API b2Manifold b2CollidePolygonAndCircle( const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

/// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsules( const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB );

/// Compute the contact manifold between an segment and a capsule
B2_API b2Manifold b2CollideSegmentAndCapsule( const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
											  b2Transform xfB );

/// Compute the contact manifold between a polygon and capsule
B2_API b2Manifold b2CollidePolygonAndCapsule( const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB,
											  b2Transform xfB );

/// Compute the contact manifold between two polygons
B2_API b2Manifold b2CollidePolygons( const b2Polygon* polygonA, b2Transform xfA, const b2Polygon* polygonB, b2Transform xfB );

/// Compute the contact manifold between an segment and a polygon
B2_API b2Manifold b2CollideSegmentAndPolygon( const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
											  b2Transform xfB );

/// Compute the contact manifold between a chain segment and a circle
B2_API b2Manifold b2CollideChainSegmentAndCircle( const b2ChainSegment* segmentA, b2Transform xfA, const b2Circle* circleB,
												  b2Transform xfB );

/// Compute the contact manifold between a chain segment and a capsule
B2_API b2Manifold b2CollideChainSegmentAndCapsule( const b2ChainSegment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
												   b2Transform xfB, b2SimplexCache* cache );

/// Compute the contact manifold between a chain segment and a rounded polygon
B2_API b2Manifold b2CollideChainSegmentAndPolygon( const b2ChainSegment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
												   b2Transform xfB, b2SimplexCache* cache );
*/
/**@}*/

/**
 * @defgroup tree Dynamic Tree
 * The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
 *
 * Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
 * This data structure may have uses in games for organizing other geometry data and may be used independently
 * of Box2D rigid body simulation.
 *
 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as AABB queries and ray casts. Leaf nodes are proxies
 * with an AABB. These are used to hold a user collision object.
 * Nodes are pooled and relocatable, so I use node indices rather than pointers.
 * The dynamic tree is made available for advanced users that would like to use it to organize
 * spatial game data besides rigid bodies.
 * @{
 */

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
typedef struct b2DynamicTree
{
	/// The tree nodes
	struct b2TreeNode* nodes;

	/// The root index
	int root;

	/// The number of nodes
	int nodeCount;

	/// The allocated node space
	int nodeCapacity;

	/// Node free list
	int freeList;

	/// Number of proxies created
	int proxyCount;

	/// Leaf indices for rebuild
	int* leafIndices;

	/// Leaf bounding boxes for rebuild
	b2AABB* leafBoxes;

	/// Leaf bounding box centers for rebuild
	b2Vec2* leafCenters;

	/// Bins for sorting during rebuild
	int* binIndices;

	/// Allocated space for rebuilding
	int rebuildCapacity;
} b2DynamicTree;

/// These are performance results returned by dynamic tree queries.
typedef struct b2TreeStats
{
	/// Number of internal nodes visited during the query
	int nodeVisits;

	/// Number of leaf nodes visited during the query
	int leafVisits;
} b2TreeStats;

/*
/// Constructing the tree initializes the node pool.
B2_API b2DynamicTree b2DynamicTree_Create( void );

/// Destroy the tree, freeing the node pool.
B2_API void b2DynamicTree_Destroy( b2DynamicTree* tree );

/// Create a proxy. Provide an AABB and a userData value.
B2_API int b2DynamicTree_CreateProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, uint64_t userData );

/// Destroy a proxy. This asserts if the id is invalid.
B2_API void b2DynamicTree_DestroyProxy( b2DynamicTree* tree, int proxyId );

/// Move a proxy to a new AABB by removing and reinserting into the tree.
B2_API void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb );

/// Enlarge a proxy and enlarge ancestors as necessary.
B2_API void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb );

/// Modify the category bits on a proxy. This is an expensive operation.
B2_API void b2DynamicTree_SetCategoryBits( b2DynamicTree* tree, int proxyId, uint64_t categoryBits );

/// Get the category bits on a proxy.
B2_API uint64_t b2DynamicTree_GetCategoryBits( b2DynamicTree* tree, int proxyId );

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
typedef bool b2TreeQueryCallbackFcn( int proxyId, uint64_t userData, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
///	@return performance data
B2_API b2TreeStats b2DynamicTree_Query( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits,
										b2TreeQueryCallbackFcn* callback, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
/// No filtering is performed.
///	@return performance data
B2_API b2TreeStats b2DynamicTree_QueryAll( const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback,
										   void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2TreeRayCastCallbackFcn( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context );

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
B2_API b2TreeStats b2DynamicTree_RayCast( const b2DynamicTree* tree, const b2RayCastInput* input, uint64_t maskBits,
										  b2TreeRayCastCallbackFcn* callback, void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2TreeShapeCastCallbackFcn( const b2ShapeCastInput* input, int proxyId, uint64_t userData, void* context );

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
B2_API b2TreeStats b2DynamicTree_ShapeCast( const b2DynamicTree* tree, const b2ShapeCastInput* input, uint64_t maskBits,
											b2TreeShapeCastCallbackFcn* callback, void* context );

/// Get the height of the binary tree.
B2_API int b2DynamicTree_GetHeight( const b2DynamicTree* tree );

/// Get the ratio of the sum of the node areas to the root area.
B2_API float b2DynamicTree_GetAreaRatio( const b2DynamicTree* tree );

/// Get the bounding box that contains the entire tree
B2_API b2AABB b2DynamicTree_GetRootBounds( const b2DynamicTree* tree );

/// Get the number of proxies created
B2_API int b2DynamicTree_GetProxyCount( const b2DynamicTree* tree );

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
B2_API int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild );

/// Get the number of bytes used by this tree
B2_API int b2DynamicTree_GetByteCount( const b2DynamicTree* tree );

/// Get proxy user data
B2_API uint64_t b2DynamicTree_GetUserData( const b2DynamicTree* tree, int proxyId );

/// Get the AABB of a proxy
B2_API b2AABB b2DynamicTree_GetAABB( const b2DynamicTree* tree, int proxyId );

/// Validate this tree. For testing.
B2_API void b2DynamicTree_Validate( const b2DynamicTree* tree );

/// Validate this tree has no enlarged AABBs. For testing.
B2_API void b2DynamicTree_ValidateNoEnlarged( const b2DynamicTree* tree );
*/
/**@}*/

/**
 * @defgroup character Character mover
 * Character movement solver
 * @{
 */

/// These are the collision planes returned from b2World_CollideMover
typedef struct b2PlaneResult
{
	/// The collision plane between the mover and a convex shape
	b2Plane plane;

	// The collision point on the shape.
	b2Vec2 point;

	/// Did the collision register a hit? If not this plane should be ignored.
	bool hit;
} b2PlaneResult;

/// These are collision planes that can be fed to b2SolvePlanes. Normally
/// this is assembled by the user from plane results in b2PlaneResult
typedef struct b2CollisionPlane
{
	/// The collision plane between the mover and some shape
	b2Plane plane;

	/// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
	/// make the plane collision soft. Usually in meters.
	float pushLimit;

	/// The push on the mover determined by b2SolvePlanes. Usually in meters.
	float push;

	/// Indicates if b2ClipVector should clip against this plane. Should be false for soft collision.
	bool clipVelocity;
} b2CollisionPlane;

/// Result returned by b2SolvePlanes
typedef struct b2PlaneSolverResult
{
	/// The translation of the mover
	b2Vec2 translation;

	/// The number of iterations used by the plane solver. For diagnostics.
	int iterationCount;
} b2PlaneSolverResult;

/*
/// Solves the position of a mover that satisfies the given collision planes.
/// @param targetDelta the desired movement from the position used to generate the collision planes
/// @param planes the collision planes
/// @param count the number of collision planes
B2_API b2PlaneSolverResult b2SolvePlanes( b2Vec2 targetDelta, b2CollisionPlane* planes, int count );

/// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
/// set to false are skipped.
B2_API b2Vec2 b2ClipVector( b2Vec2 vector, const b2CollisionPlane* planes, int count );
*/
