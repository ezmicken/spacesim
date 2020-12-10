package spacesim

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

func RectOverlap(a, b Rect) fixpoint.Q16 {
  overlapX := fixpoint.Max(a.Min.X, b.Min.X)
  overlapY := fixpoint.Max(a.Min.Y, b.Min.Y)
  overlapW := fixpoint.Min(a.Max.X, b.Max.X).Sub(overlapX)
  overlapH := fixpoint.Min(a.Max.Y, b.Max.Y).Sub(overlapY)

  return overlapW.Mul(overlapH)
}
