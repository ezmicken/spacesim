package main

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

