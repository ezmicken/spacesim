package spacesim

import(
  "testing"
  "github.com/ezmicken/fixpoint"
)

func TestRectOverlap(t *testing.T) {
  // test non-overlapping rectangles.
  zero := fixpoint.ZeroQ16
  one := fixpoint.OneQ16
  two := fixpoint.TwoQ16
  three := two.Add(one)
  four := two.Mul(two)


  // Test rectangles that do not overlap
  leftRect := NewRect(zero, zero, two, two)
  rightRect := NewRect(three, zero, two, two)
  noOverlap := RectOverlap(leftRect, rightRect)
  if noOverlap != zero {
    t.Logf("no overlap case found overlap of %v", noOverlap.Float())
    t.Fail()
  }

  // Test that one rectangle inside another
  // overlaps by the correct amount
  outsideRect := NewRect(zero, zero, four, four)
  insideRect := NewRect(one, one, two, two)
  fullOverlap := RectOverlap(outsideRect, insideRect)
  if fullOverlap != four {
    t.Logf("full overlap with different rectangles found unexpected overlap %v", fullOverlap.Float())
    t.Fail()
  }

  // Test that two identical rectangles overlap
  rect1 := NewRect(zero, zero, two, two)
  rect2 := NewRect(zero, zero, two, two)
  fullOverlap2 := RectOverlap(rect1, rect2)
  if fullOverlap2 != four {
    t.Logf("full overlap with identical rectangles found undexpected overlap %v", fullOverlap.Float())
    t.Fail()
  }

  // Test that a rectangle overlaps one side
  leftRect = NewRect(zero, one, two, two)
  rightRect = NewRect(one, zero, four, four)
  leftOverlap := RectOverlap(leftRect, rightRect)
  if leftOverlap != two {
    t.Logf("overlap with one side found unexpected overlap %v", leftOverlap.Float())
    t.Fail()
  }

  // Test that rectangle corners overlap
  leftRect = NewRect(zero, one, two, two)
  rightRect = NewRect(one, zero, two, two)
  cornerOverlap := RectOverlap(leftRect, rightRect)
  if cornerOverlap != one {
    t.Logf("corner overlap found unexpected overlap %v", cornerOverlap.Float())
    t.Fail()
  }
}
