package spacesim

import(
  "testing"
  "github.com/ezmicken/fixpoint"
  "github.com/ezmicken/uuint16"
)

var ts fixpoint.Q16 = fixpoint.Q16FromInt32(33)
var scale fixpoint.Q16 = fixpoint.Q16FromInt32(32)

func TestSerializeState(t *testing.T) {
  sim := NewSimulation(ts, scale)
  id, err := uuint16.Rent()
  if err != nil {
    t.Log("uuint16.Rent failed...")
    t.Fail()
  }

  sim.AddControlledBody(id, int32(1234), int32(4321), int32(1))
  cb := sim.GetControlledBody(id)

  for i := 0; i < 100; i++ {
    cb.PushInput(uint16(i + 12), byte(4))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.Advance(i)
  }

  data := make([]byte, 1024)
  head := sim.SerializeState(data, 0)

  // expect 47 bytes of data
  if head != 47 {
    t.Logf("head was not 47, it was %v", head)
    t.Fail()
  }
}

func TestSerializeState2(t *testing.T) {
  sim := NewSimulation(ts, scale)
  id, err := uuint16.Rent()
  if err != nil {
    t.Log("uuint16.Rent failed...")
    t.Fail()
  }
  id2, err := uuint16.Rent()
  if err != nil {
    t.Log("uuint16.Rend failed second time...")
    t.Fail()
  }

  sim.AddControlledBody(id, int32(1234), int32(4321), int32(1))
  sim.AddControlledBody(id2, int32(4321), int32(1234), int32(1))
  cb := sim.GetControlledBody(id)
  cb2 := sim.GetControlledBody(id2)

  for i := 0; i < 100; i++ {
    cb.PushInput(uint16(i + 12), byte(4))
    cb2.PushInput(uint16(i + 12), byte(2))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.AdvanceControlledBody(id2, uint16(i))
    sim.Advance(i)
  }

  data := make([]byte, 1024)
  head := sim.SerializeState(data, 0)

  // expect 90 bytes of data
  if head != 90 {
    t.Logf("head was not 90, it was %v", head)
    t.Fail()
  }
}

// This runs ReplaceControlledBody instead of Add.
func TestSerializeState3(t *testing.T) {
  sim := NewSimulation(ts, scale)
  id, err := uuint16.Rent()
  if err != nil {
    t.Log("uuint16.Rent failed...")
    t.Fail()
  }

  sim.ReplaceControlledBody(id, uint16(0), uint16(0), int32(1), int32(2), int32(3), int32(4), int32(5), int32(6))
  cb := sim.GetControlledBody(id)

  for i := 0; i < 100; i++ {
    cb.PushInput(uint16(i + 12), byte(4))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.Advance(i)
  }

  data := make([]byte, 1024)
  head := sim.SerializeState(data, 0)

  // expect 47 bytes of data
  if head != 47 {
    t.Logf("head was not 47, it was %v", head)
    t.Fail()
  }
}
