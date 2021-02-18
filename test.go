package spacesim

import(
  "github.com/ezmicken/fixpoint"
)

var timestep fixpoint.Q16 = fixpoint.Q16FromInt32(33)
var scale fixpoint.Q16 = fixpoint.Q16FromInt32(32)

var randomHT HistoricalTransform = HistoricalTransform {
  Seq: 500,
  Angle: 0,
  AngleDelta: 0,
  Position: fixpoint.ZeroVec3Q16,
  Velocity: fixpoint.ZeroVec3Q16,
  VelocityDelta: fixpoint.ZeroVec3Q16,
}
