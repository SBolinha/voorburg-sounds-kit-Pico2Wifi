// Local formatter checks; no TTN traffic or private configuration.
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const context = {};
vm.createContext(context);
vm.runInContext(fs.readFileSync(__dirname + '/ttn_uplink_payload_formatter.js', 'utf8'), context);
const decode = bytes => context.decodeUplink({bytes, fPort:1});
const five = [143,78,165,30,146];
const legacy = decode(five);
assert.strictEqual(legacy.data.mean,71.5);
assert.strictEqual(legacy.data.min,39);
assert.strictEqual(legacy.data.max,82.5);
assert.strictEqual(legacy.data.stddev,3.75);
assert.strictEqual(legacy.data.leq,73);
assert.strictEqual(legacy.data.at,undefined);
for (const seconds of [1791532800,2147483648,4294967295]) {
  const bytes = five.concat([seconds & 255, Math.floor(seconds/256)&255,
    Math.floor(seconds/65536)&255, Math.floor(seconds/16777216)&255]);
  const decoded = decode(bytes);
  assert.strictEqual(decoded.data.at,new Date(seconds*1000).toISOString());
  assert.strictEqual(decoded.data.leq,73);
}
assert(decode(five.concat([0,0,0,0])).errors.length);
for(const length of [0,4,6,8,10]) assert(decode(new Array(length).fill(0)).errors.length);
console.log('Payload formatter: legacy five values, UTC nine-byte format, post-2038 unsigned timestamp and invalid lengths passed');
