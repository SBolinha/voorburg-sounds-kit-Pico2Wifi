// Paste into the end device's TTN uplink payload formatter.
// Byte order: mean/min/max in unsigned Q7.1, standard deviation in unsigned
// Q5.3, then LEQ in unsigned Q7.1. Bytes 5..8 are the measurement window's
// end time: unsigned Unix seconds, little-endian. Legacy five-byte payloads
// remain accepted. Levels are relative and uncalibrated.
function decodeUplink(input) {
  var bytes = input.bytes || [];
  if (bytes.length !== 5 && bytes.length !== 9) {
    return {
      errors: ["Expected a 5- or 9-byte Voorburg sound-level payload; got " + bytes.length + " bytes."],
    };
  }

  var data = {
      mean: bytes[0] / 2,
      min: bytes[1] / 2,
      max: bytes[2] / 2,
      stddev: bytes[3] / 8,
      leq: bytes[4] / 2,
  };
  if (bytes.length === 9) {
    // Arithmetic avoids JavaScript's signed 32-bit bitwise result after 2038.
    var seconds = bytes[5] + bytes[6] * 256 + bytes[7] * 65536 + bytes[8] * 16777216;
    if (seconds === 0) return { errors: ["Measurement UTC timestamp is missing."] };
    data.at = new Date(seconds * 1000).toISOString();
  }
  return { data: data };
}
