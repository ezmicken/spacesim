package spacesim

import(
  "testing"
  "github.com/ezmicken/fixpoint"
)

func TestAddBlock(t *testing.T) {
  b := NewBody(testControlledBodyInfo, thirtyTwo)
  cb := NewControlledBody(1, fixpoint.ZeroQ16, twenty, thirtyTwo, b)
  for i := int32(0); i < 300; i++ {
    cb.AddBlock(i, i)
  }
}

func TestAdvance(t *testing.T) {
  b := NewBody(testControlledBodyInfo, thirtyTwo)
  cb := NewControlledBody(9, fixpoint.Q16FromFloat(0.36), twenty, scale, b)

  var ht HistoricalTransform
  ht.Seq = uint16(0)
  ht.Angle = 0
  ht.AngleDelta = 0
  ht.Position = fixpoint.ZeroVec3Q16
  ht.Velocity = fixpoint.ZeroVec3Q16
  ht.VelocityDelta = fixpoint.ZeroVec3Q16

  b.Initialize(ht)

  testSeq := uint16(12)
  for i := uint16(0); i < 640; i++ {
    cb.PushInput(testSeq, byte(4))
    testSeq++
    cb.GetBody().Advance(i)
    cb.Advance(i)
  }
}
