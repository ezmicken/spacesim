package main

import(
  "github.com/ezmicken/fixpoint"
)

type Point struct {
  X fixpoint.Q16
  Y fixpoint.Q16
}

type Rect struct {
  Min Point
  Max Point
  W   fixpoint.Q16
  H   fixpoint.Q16
}

func NewRect(x, y, w, h fixpoint.Q16) Rect {
  var r Rect
  r.Min = Point{X: x, Y: y}
  r.Max = Point{X: x.Add(w), Y: y.Add(h)}
  r.W = w
  r.H = h
  return r
}
