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

func (r Rect) Size() fixpoint.Q16 {
  return r.W.Mul(r.H)
}

var InvalidRect Rect = NewRect(fixpoint.OneQ16.Neg(), fixpoint.OneQ16.Neg(), fixpoint.OneQ16.Neg(), fixpoint.OneQ16.Neg())

func NewRect(x, y, w, h fixpoint.Q16) Rect {
  var r Rect
  r.Min = Point{X: x, Y: y}
  r.Max = Point{X: x.Add(w), Y: y.Add(h)}
  r.W = w
  r.H = h
  return r
}

func RectOverlap(a, b Rect) fixpoint.Q16 {
  area := fixpoint.ZeroQ16

  left := b.Min.X.Sub(a.Max.X)
  top := b.Max.Y.Sub(a.Min.Y)
  right := b.Max.X.Sub(a.Min.X)
  bottom := b.Min.Y.Sub(a.Max.Y)

  if !(left.N > 0 || right.N < 0 || top.N < 0 || bottom.N > 0) {
    overlapX := fixpoint.Max(a.Min.X, b.Min.X)
    overlapY := fixpoint.Max(a.Min.Y, b.Min.Y)
    overlapW := fixpoint.Min(a.Max.X, b.Max.X).Sub(overlapX)
    overlapH := fixpoint.Min(a.Max.Y, b.Max.Y).Sub(overlapY)
    area = overlapW.Mul(overlapH)
  }

  return area
}
