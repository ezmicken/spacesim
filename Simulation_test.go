package spacesim

import(
  "testing"
  "github.com/ezmicken/uuint16"
)

func TestSerializeState(t *testing.T) {
  sim := NewSimulation(timestep, scale)
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

func TestIntegration(t *testing.T) {
  sim := NewSimulation(timestep, scale)

  // Rent two ids
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

  // Add two bodies
  sim.AddControlledBody(id, int32(1234), int32(4321), int32(1))
  sim.AddControlledBody(id2, int32(4321), int32(1234), int32(1))
  cb := sim.GetControlledBody(id)
  cb2 := sim.GetControlledBody(id2)

  // Apply input to two bodies and advance
  for i := 0; i < 100; i++ {
    cb.PushInput(uint16(i + 12), byte(4))
    cb2.PushInput(uint16(i + 12), byte(2))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.AdvanceControlledBody(id2, uint16(i))
    sim.Advance(i)
  }

  // Serialize the state
  data := make([]byte, 1024)
  head := sim.SerializeState(data, 0)

  // expect 90 bytes of data
  if head != 90 {
    t.Logf("head was not 90, it was %v", head)
    t.Fail()
  }

  ht := randomHT
  ht.Seq = 0
  // Rewind and overwrite state of both bodies.
  sim.Rewind(ht.Seq + 1)

  sim.OverwriteState(ht.Seq, id, uint16(0), uint16(0), int32(0), int32(0), int32(0), int32(0), int32(0), int32(0))
  sim.OverwriteState(ht.Seq, id2, uint16(0), uint16(0), int32(0), int32(0), int32(0), int32(0), int32(0), int32(0))

  for i := int(ht.Seq + 1); i < 101; i++ {
    sim.AdvanceControlledBody(id, uint16(i))
    sim.AdvanceControlledBody(id2, uint16(i))
    sim.Advance(i)
  }

  // Get the next state
  cb1state := sim.PeekState(id)
  cb2state := sim.PeekState(id2)

  // Expect to have rewound to the correct seq
  if cb1state.Seq != ht.Seq + 100 {
    t.Logf("rewind failed cb1 %v was not %v", cb1state.Seq, ht.Seq)
    t.Fail()
  }

  if cb2state.Seq != ht.Seq + 100 {
    t.Logf("rewind failed cb2 %v was not %v", cb1state.Seq, ht.Seq)
    t.Fail()
  }
}
