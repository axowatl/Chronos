//#region Ids
/*
 * These ids serve as handles to internal Box2D objects.
 * These should be considered opaque data and passed by value.
 * Include this header if you need the id types and not the whole Box2D API.
 * All ids are considered null if initialized to zero.
 *
 * For example in C++:
 *
 * @code{.cxx}
 * b2WorldId worldId = {};
 * @endcode
 *
 * Or in C:
 *
 * @code{.c}
 * b2WorldId worldId = {0};
 * @endcode
 *
 * These are both considered null.
 *
 * @warning Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
 * @warning You should use ids to access objects in Box2D. Do not access files within the src folder. Such usage is unsupported.
 * @{
 */

/**
 * World id references a world instance. This should be treated as an opaque handle.
 */
export class b2WorldId
{
    /**
     * 
     * @param {number} index1 
     * @param {number} generation 
     */
    constructor(index1, generation) {
        this.index1 = index1;
        this.generation = generation;
    }
}

/**
 * Body id references a body instance. This should be treated as an opaque handle.
 */
export class b2BodyId
{
    /**
     * 
     * @param {number} index1 
     * @param {number} world0 
     * @param {number} generation 
     */
    constructor(index1, world0, generation) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/**
 * Shape id references a shape instance. This should be treated as an opaque handle.
 */
export class b2ShapeId
{
	/**
     * 
     * @param {number} index1 
     * @param {number} world0 
     * @param {number} generation 
     */
    constructor(index1, world0, generation) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/**
 * Chain id references a chain instances. This should be treated as an opaque handle.
 */
export class b2ChainId
{
	/**
     * 
     * @param {number} index1 
     * @param {number} world0 
     * @param {number} generation 
     */
    constructor(index1, world0, generation) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/**
 * Joint id references a joint instance. This should be treated as an opaque handle.
 */
export class b2JointId
{
	/**
     * 
     * @param {number} index1 
     * @param {number} world0 
     * @param {number} generation 
     */
    constructor(index1, world0, generation) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

/**
 * Contact id references a contact instance. This should be treated as an opaque handled.
 */
export class b2ContactId
{
	/**
     * 
     * @param {number} index1 
     * @param {number} world0 
     * @param {number} padding 
     * @param {number} generation 
     */
    constructor(index1, world0, padding, generation) {
        this.index1 = index1;
        this.world0 = world0;
        this.padding = padding;
        this.generation = generation;
    }
}


/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
export const b2_nullWorldId = new b2WorldId(0, 0);
export const b2_nullBodyId = new b2BodyId(0, 0, 0);
export const b2_nullShapeId = new b2ShapeId(0, 0, 0);
export const b2_nullChainId = new b2ChainId(0, 0, 0);
export const b2_nullJointId = new b2JointId(0, 0, 0);
export const b2_nullContactId = new b2ContactId(0, 0, 0, 0);

/**
 * Macro to determine if any id is null.
 * @param {b2WorldId | b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ContactId} id 
 * @returns {boolean}
 */
export function IS_NULL(id) { return id.index1 == 0; }

/**
 * Macro to determine if any id is non-null.
 * @param {b2WorldId | b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ContactId} id 
 * @returns {boolean}
 */
export function IS_NON_NULL(id) { return id.index1 != 0; }

/**
 * Compare two ids for equality. Doesn't work for b2WorldId. Don't mix types.
 * @param {b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ContactId} id1 
 * @param {b2BodyId | b2ShapeId | b2ChainId | b2JointId | b2ContactId} id2 
 * @returns {boolean}
 */
export function ID_EQUALS(id1, id2) { return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.generation == id2.generation; }

/**
 * Store a world id into a uint32_t.
 * @param {b2WorldId} id
 * @returns {number}
 */
export function b2StoreWorldId(id)
{
	return (id.index1 << 16) | id.generation;
}

/**
 * Load a uint32_t into a world id.
 * @param {number} x 
 * @returns {b2WorldId}
 */
export function b2LoadWorldId(x)
{
    return new b2WorldId(x >>> 16, x & 0xFFFF);
}

/**
 * Store a body id into a 64-bit integer using BigInt.
 * @param {b2BodyId} id 
 * @returns {BigInt}
 */
export function b2StoreBodyId(id) {
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a body id using BigInt.
 * @param {BigInt} x 
 * @returns {b2BodyId}
 */
export function b2LoadBodyId(x) {
    const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new b2BodyId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a shape id into a 64-bit integer using BigInt.
 * @param {b2ShapeId} id 
 * @returns {BigInt}
 */
export function b2StoreShapeId(id)
{
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a shape id using BigInt.
 * @param {BigInt} x 
 * @returns {b2ShapeId}
 */
export function b2LoadShapeId(x)
{
	const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new b2ShapeId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a chain id into a 64-bit integer using BigInt.
 * @param {b2ChainId} id 
 * @returns {BigInt}
 */
export function b2StoreChainId(id)
{
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a chain id using BigInt.
 * @param {BigInt} x 
 * @returns {b2ChainId}
 */
export function b2LoadChainId(x)
{
	const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new b2ChainId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a joint id into a 64-bit integer using BigInt.
 * @param {b2JointId} id 
 * @returns {BigInt}
 */
export function b2StoreJointId(id)
{
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a joint id using BigInt.
 * @param {BigInt} x 
 * @returns {b2JointId}
 */
export function b2LoadJointId(x)
{
	const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new b2JointId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a contact id into 16 bytes
 * @param {b2ContactId} id 
 * @returns {number[]}
 */
export function b2StoreContactId(id)
{
    let values = new Array(3);
	values[0] = id.index1;
	values[1] = id.world0;
	values[2] = id.generation;
    return values;
}

/**
 * Load a two uint64_t into a contact id.
 * @param {number[]} values 
 * @returns {b2ContactId}
 */
export function b2LoadContactId(values)
{
	return new b2ContactId(values[0], values[1], 0, values[2]);
}
//#endregion Ids
