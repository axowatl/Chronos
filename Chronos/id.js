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
 * WorldId worldId = {};
 * @endcode
 *
 * Or in C:
 *
 * @code{.c}
 * WorldId worldId = {0};
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
export class WorldId
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
export class BodyId
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
export class ShapeId
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
export class ChainId
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
export class JointId
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
export class ContactId
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
export const nullWorldId = new WorldId(0, 0);
export const nullBodyId = new BodyId(0, 0, 0);
export const nullShapeId = new ShapeId(0, 0, 0);
export const nullChainId = new ChainId(0, 0, 0);
export const nullJointId = new JointId(0, 0, 0);
export const nullContactId = new ContactId(0, 0, 0, 0);

/**
 * Macro to determine if any id is null.
 * @param {WorldId | BodyId | ShapeId | ChainId | JointId | ContactId} id 
 * @returns {boolean}
 */
export function IS_NULL(id) { return id.index1 == 0; }

/**
 * Macro to determine if any id is non-null.
 * @param {WorldId | BodyId | ShapeId | ChainId | JointId | ContactId} id 
 * @returns {boolean}
 */
export function IS_NON_NULL(id) { return id.index1 != 0; }

/**
 * Compare two ids for equality. Doesn't work for WorldId. Don't mix types.
 * @param {BodyId | ShapeId | ChainId | JointId | ContactId} id1 
 * @param {BodyId | ShapeId | ChainId | JointId | ContactId} id2 
 * @returns {boolean}
 */
export function ID_EQUALS(id1, id2) { return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.generation == id2.generation; }

/**
 * Store a world id into a uint32_t.
 * @param {WorldId} id
 * @returns {number}
 */
export function StoreWorldId(id)
{
	return (id.index1 << 16) | id.generation;
}

/**
 * Load a uint32_t into a world id.
 * @param {number} x 
 * @returns {WorldId}
 */
export function LoadWorldId(x)
{
    return new WorldId(x >>> 16, x & 0xFFFF);
}

/**
 * Store a body id into a 64-bit integer using BigInt.
 * @param {BodyId} id 
 * @returns {BigInt}
 */
export function StoreBodyId(id) {
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a body id using BigInt.
 * @param {BigInt} x 
 * @returns {BodyId}
 */
export function LoadBodyId(x) {
    const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new BodyId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a shape id into a 64-bit integer using BigInt.
 * @param {ShapeId} id 
 * @returns {BigInt}
 */
export function StoreShapeId(id)
{
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a shape id using BigInt.
 * @param {BigInt} x 
 * @returns {ShapeId}
 */
export function LoadShapeId(x)
{
	const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new ShapeId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a chain id into a 64-bit integer using BigInt.
 * @param {ChainId} id 
 * @returns {BigInt}
 */
export function StoreChainId(id)
{
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a chain id using BigInt.
 * @param {BigInt} x 
 * @returns {ChainId}
 */
export function LoadChainId(x)
{
	const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new ChainId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a joint id into a 64-bit integer using BigInt.
 * @param {JointId} id 
 * @returns {BigInt}
 */
export function StoreJointId(id)
{
    const index1 = BigInt(id.index1);
    const world0 = BigInt(id.world0);
    const generation = BigInt(id.generation);

    return (index1 << 32n) | (world0 << 16n) | generation;
}

/**
 * Load a 64-bit integer into a joint id using BigInt.
 * @param {BigInt} x 
 * @returns {JointId}
 */
export function LoadJointId(x)
{
	const index1 = x >> 32n;
    const world0 = (x >> 16n) & 0xFFFFn;
    const generation = x & 0xFFFFn;

    return new JointId(Number(index1), Number(world0), Number(generation));
}

/**
 * Store a contact id into 16 bytes
 * @param {ContactId} id 
 * @returns {number[]}
 */
export function StoreContactId(id)
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
 * @returns {ContactId}
 */
export function LoadContactId(values)
{
	return new ContactId(values[0], values[1], 0, values[2]);
}
//#endregion Ids
