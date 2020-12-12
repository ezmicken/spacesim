package spacesim

import(
  "testing"
  "github.com/ezmicken/fixpoint"
)

func TestAddBlock(t *testing.T) {
  sim := NewSimulation(fixpoint.ZeroQ16, fixpoint.Q16FromInt32(32))
  cb := NewControlledBody(1, fixpoint.ZeroQ16, fixpoint.ZeroQ16, sim)
  for i := int32(0); i < 300; i++ {
    cb.AddBlock(i, i)
  }
}
