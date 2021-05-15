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

  sim.AddControlledBody(id, int32(1234), int32(4321), testControlledBodyInfo)
  cb := sim.GetControlledBody(id)

  for i := 0; i < 100; i++ {
    cb.PushInput(uint16(i + 12), byte(4))
    sim.AdvanceBody(id, uint16(i))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.Advance(i)
  }

  data := make([]byte, 1024)
  head := sim.SerializeState(data, 0)

  // 12 inputs
  // 1 player
  // 1 body
  // expect 53 bytes of data
  if head != 52 {
    t.Logf("head was not 52, it was %v", head)
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
  sim.AddControlledBody(id, int32(1234), int32(4321), testControlledBodyInfo)
  sim.AddControlledBody(id2, int32(4321), int32(1234), testControlledBodyInfo)
  cb := sim.GetControlledBody(id)
  cb2 := sim.GetControlledBody(id2)

  // Apply input to two bodies and advance
  for i := 0; i < 100; i++ {
    cb.PushInput(uint16(i + 12), byte(4))
    cb2.PushInput(uint16(i + 12), byte(2))
    sim.AdvanceBody(id, uint16(i))
    sim.AdvanceBody(id2, uint16(i))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.AdvanceControlledBody(id2, uint16(i))
    sim.Advance(i)
  }

  // Serialize the state
  data := make([]byte, 1024)
  head := sim.SerializeState(data, 0)

  // expect 90 bytes of data
  if head != 97 {
    t.Logf("head was not 97, it was %v", head)
    t.Fail()
  }

  ht := randomHT
  ht.Seq = 0
  // Rewind and overwrite state of both bodies.
  sim.Rewind(ht.Seq + 1)

  sim.OverwriteState(ht.Seq, id, uint16(0), uint16(0), int32(0), int32(0), int32(0), int32(0), int32(0), int32(0))
  sim.OverwriteState(ht.Seq, id2, uint16(0), uint16(0), int32(0), int32(0), int32(0), int32(0), int32(0), int32(0))

  for i := int(ht.Seq + 1); i < 101; i++ {
    sim.AdvanceBody(id, uint16(i))
    sim.AdvanceBody(id2, uint16(i))
    sim.AdvanceControlledBody(id, uint16(i))
    sim.AdvanceControlledBody(id2, uint16(i))
    sim.Advance(i)
  }

  // Get the next state
  cb1state := cb.GetBody().GetState()
  cb2state := cb2.GetBody().GetState()

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
