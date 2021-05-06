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
  area := fixpoint.ZeroQ16


  // Test rectangles that do not overlap
  leftRect := NewRect(zero, zero, two, two)
  rightRect := NewRect(three, zero, two, two)
  noOverlap := RectOverlap(leftRect, rightRect)
  if noOverlap != InvalidRect {
    t.Logf("no overlap case found overlap")
    t.Fail()
  }

  leftRect = NewRect(thirtyTwo.Neg(), thirtyTwo, thirtyTwo, thirtyTwo)
  rightRect = NewRect(twentyNine.Neg(), thirtyTwo, thirtyTwo, thirtyTwo)
  negOverlap := RectOverlap(leftRect, rightRect)
  if negOverlap == InvalidRect {
    t.Logf("negative overlap case did not find overlap")
    t.Fail()
  }

  // Test that one rectangle inside another
  // overlaps by the correct amount
  outsideRect := NewRect(zero, zero, four, four)
  insideRect := NewRect(one, one, two, two)
  fullOverlap := RectOverlap(outsideRect, insideRect)
  area = fullOverlap.W.Mul(fullOverlap.H)
  if area != four {
    t.Logf("full overlap with different rectangles found unexpected overlap %v", area.Float())
    t.Fail()
  }

  // Test that two identical rectangles overlap
  rect1 := NewRect(zero, zero, two, two)
  rect2 := NewRect(zero, zero, two, two)
  fullOverlap2 := RectOverlap(rect1, rect2)
  area = fullOverlap2.W.Mul(fullOverlap2.H)
  if area != four {
    t.Logf("full overlap with identical rectangles found undexpected overlap %v", area.Float())
    t.Fail()
  }

  // Test that a rectangle overlaps one side
  leftRect = NewRect(zero, one, two, two)
  rightRect = NewRect(one, zero, four, four)
  leftOverlap := RectOverlap(leftRect, rightRect)
  area = leftOverlap.W.Mul(leftOverlap.H)
  if area != two {
    t.Logf("overlap with one side found unexpected overlap %v", area.Float())
    t.Fail()
  }

  // Test that rectangle corners overlap
  leftRect = NewRect(zero, one, two, two)
  rightRect = NewRect(one, zero, two, two)
  cornerOverlap := RectOverlap(leftRect, rightRect)
  area = cornerOverlap.W.Mul(cornerOverlap.H)
  if area != one {
    t.Logf("corner overlap found unexpected overlap %v", area.Float())
    t.Fail()
  }
}
