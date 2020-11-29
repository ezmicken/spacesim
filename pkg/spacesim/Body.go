package main

import(
  "github.com/ezmicken/fixpoint"
)

type Body struct {
  Angle             int32
  NextAngle         int32

  Pos               fixpoint.Vec3Q16
  NextPos           fixpoint.Vec3Q16
  Vel               fixpoint.Vec3Q16

  dead  bool
}

func NewBody() *Body {
  var b Body
  b.Angle = 0
  b.NextAngle = 0
  b.NextPos = fixpoint.ZeroVec3Q16
  b.Pos = fixpoint.ZeroVec3Q16
  b.Vel = fixpoint.ZeroVec3Q16
  b.dead = false
  return &b
}

func (b *Body) Advance(seq uint16) {
  b.Angle = b.NextAngle
  b.Pos = b.NextPos
}

func(b *Body) IsDead() bool {
  return b.dead
}

func (b *Body) Kill() {
  b.dead = true
}
