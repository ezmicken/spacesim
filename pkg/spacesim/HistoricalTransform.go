package main

import(
  "github.com/ezmicken/fixpoint"
)

type HistoricalTransform struct {
  Seq           int
  Angle         int
  AngleDelta    int
  Position      fixpoint.Vec3Q16
  Velocity      fixpoint.Vec3Q16
  VelocityDelta fixpoint.Vec3Q16
}

