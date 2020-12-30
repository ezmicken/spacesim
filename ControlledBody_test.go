package spacesim

import(
  "testing"
  "github.com/ezmicken/fixpoint"
)

func TestAddBlock(t *testing.T) {
  sim := NewSimulation(fixpoint.ZeroQ16, fixpoint.Q16FromInt32(32))
  cb := NewControlledBody(1, 0, fixpoint.ZeroQ16, fixpoint.ZeroQ16, sim)
  for i := int32(0); i < 300; i++ {
    cb.AddBlock(i, i)
  }
}

func TestAdvance(t *testing.T) {
  sim := NewSimulation(fixpoint.ZeroQ16, fixpoint.Q16FromInt32(32))
  cb := NewControlledBody(9, 0, fixpoint.Q16FromFloat(0.36), fixpoint.Q16FromFloat(20), sim)

  var ht HistoricalTransform
  ht.Seq = uint16(0)
  ht.Angle = 0
  ht.AngleDelta = 0
  ht.Position = fixpoint.ZeroVec3Q16
  ht.Velocity = fixpoint.ZeroVec3Q16
  ht.VelocityDelta = fixpoint.ZeroVec3Q16

  cb.Initialize(ht)

  testSeq := uint16(12)
  for i := uint16(0); i < 640; i++ {
    cb.PushInput(testSeq, byte(4))
    testSeq++
    cb.Advance(i)
  }
}
