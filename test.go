package spacesim

import(
  "github.com/ezmicken/fixpoint"
)

var timestep fixpoint.Q16 = fixpoint.Q16FromInt32(33)
var scale fixpoint.Q16 = fixpoint.Q16FromInt32(32)

// some number constants
var zero        fixpoint.Q16  = fixpoint.ZeroQ16
var ten         fixpoint.Q16  = fixpoint.Q16FromInt32(10)
var thirtyTwo   fixpoint.Q16  = fixpoint.Q16FromInt32(32)
var thirtyFour  fixpoint.Q16  = fixpoint.Q16FromInt32(34)
var fourtyEight fixpoint.Q16  = fixpoint.Q16FromInt32(48)
var fifty       fixpoint.Q16  = fixpoint.Q16FromInt32(50)
var sixtyFour   fixpoint.Q16  = fixpoint.Q16FromInt32(64)
var eighty      fixpoint.Q16  = fixpoint.Q16FromInt32(80)
var oneSixty    fixpoint.Q16  = fixpoint.Q16FromInt32(160)

var randomHT HistoricalTransform = HistoricalTransform {
  Seq: 500,
  Angle: 0,
  AngleDelta: 0,
  Position: fixpoint.ZeroVec3Q16,
  Velocity: fixpoint.ZeroVec3Q16,
  VelocityDelta: fixpoint.ZeroVec3Q16,
}
