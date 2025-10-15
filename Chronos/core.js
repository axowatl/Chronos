/**
 * @readonly Set using SetLengthUnitsPerMeter(value)
 */
export let lengthUnitsPerMeter = 1; // "Private" backing variable
export const SECRET_COOKIE = 1152023;

/**
 * Sets lengthUnitsPerMeter
 * @param {number} value Greater than 0
 */
export function SetLengthUnitsPerMeter(value) {
	if (value > 0) {
		lengthUnitsPerMeter = value;
	}
}



// Current problem is that I can't use "lengthUnitsPerMeter" in "constants.js"
