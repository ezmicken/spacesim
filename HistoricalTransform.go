package spacesim

import(
  "github.com/ezmicken/fixpoint"
)

type HistoricalTransform struct {
  Seq           uint16
  Angle         int32
  AngleDelta    int32
  Position      fixpoint.Vec3Q16
  Velocity      fixpoint.Vec3Q16
  VelocityDelta fixpoint.Vec3Q16
}

var InvalidHistoricalTransform HistoricalTransform = HistoricalTransform{
  uint16(0),
  int32(-1),
  int32(-1),
  fixpoint.ZeroVec3Q16,
  fixpoint.ZeroVec3Q16,
  fixpoint.ZeroVec3Q16,
}
