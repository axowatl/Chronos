let _lengthUnitsPerMeter = 1; // "Private" backing variable

Object.defineProperty(window, 'lengthUnitsPerMeter', {
  get() {
    return _lengthUnitsPerMeter;
  },
  set(newValue) {
    if (typeof newValue !== 'number' || newValue <= 0) {
      // Throw an error if validation fails
      throw new Error("Invalid value assigned to 'lengthUnitsPerMeter': value must be a number greater than zero.");
    }
    _lengthUnitsPerMeter = newValue;
    console.log(`'lengthUnitsPerMeter' is now ${newValue}`);
  },
  // Makes the property visible when enumerating window properties
  enumerable: true,
  // Allows the property to be deleted later
  configurable: true
});

// Current problem is that I can't use "lengthUnitsPerMeter" in "constants.js"
