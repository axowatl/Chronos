import { lengthUnitsPerMeter } from "./core"
import { PI } from "./math_functions";

export const HUGE = 100000 * lengthUnitsPerMeter;

/**
 * Maximum parallel workers. Used to size some static arrays.
 */
export const MAX_WORKERS = 8;

/**
 * Maximum number of colors in the constraint graph. Constraints that cannot
 * find a color are added to the overflow set which are solved single-threaded.
 * The compound barrel benchmark has minor overflow with 24 colors
 */
export const GRAPH_COLOR_COUNT = 24;

/**
 * A small length used as a collision and constraint tolerance. Usually it is
 * chosen to be numerically significant, but visually insignificant. In meters.
 * Normally this is 0.5cm.
 * @warning modifying this can have a significant impact on stability
 */
export const LINEAR_SLOP = 0.005 * lengthUnitsPerMeter;

/**
 * Maximum number of simultaneous worlds that can be allocated
 */
export const MAX_WORLDS = 128;

/**
 * The maximum rotation of a body per time step. This limit is very large and is used
 * to prevent numerical problems. You shouldn't need to adjust this.
 * @warning increasing this to 0.5f * pi or greater will break continuous collision.
 */
export const MAX_ROTATION = 0.25 * PI;

/**
 * Box2D uses limited speculative collision. This reduces jitter.
 * Normally this is 2cm.
 * @warning modifying this can have a significant impact on performance and stability
 */
export const SPECULATIVE_DISTANCE = 4 * LINEAR_SLOP;

/**
 * This is used to fatten AABBs in the dynamic tree. This allows proxies
 * to move by a small amount without triggering a tree adjustment. This is in meters.
 * Normally this is 5cm.
 * @warning modifying this can have a significant impact on performance
 */
export const AABB_MARGIN = 0.05 * lengthUnitsPerMeter;

/**
 * The time that a body must be still before it will go to sleep. In seconds.
 */
export const TIME_TO_SLEEP = 0.5;

export const TreeNodeFlags = {
	allocatedNode: 0x0001,
	enlargedNode: 0x0002,
	leafNode: 0x0004,
};
