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
  axMax := a.X.Add(a.W)
  ayMax := a.Y.Add(a.H)
  bxMax := b.X.Add(b.W)
  byMax := b.Y.Add(b.H)

  overlapX := fixpoint.Max(a.X, b.X)
  overlapY := fixpoint.Max(a.Y, b.Y)
  overlapW := fixpoint.Min(axMax, bxMax).Sub(overlapX)
  overlapH := fixpoint.Min(ayMax, byMax).Sub(overlapY)

  return overlapW.Mul(overlapH)
}
