package main

import(
  "github.com/ezmicken/fixpoint"
)

var sim *Simulation

// export Initialize
func Initialize(ts int32) {
  sim = NewSimulation(fixpoint.Q16FromInt32(ts))
}

// export AddControlledBody
func AddControlledBody(id uint16) {
  sim.AddControlledBody(id)
}

// export RemoveControlledBody
func RemoveControlledBody(id uint16) {
  sim.RemoveControlledBody(id)
}

// export PushInput
func PushInput(id uint16, tick uint16, moveshoot byte) {
  cb := sim.GetControlledBody(id)
  if cb != nil {
    cb.InputToState(tick, moveshoot)
  }
}

// export AdvanceSim
func AdvanceSim(seq uint16) {
  sim.Advance(int(seq))
}

// export AdvanceBody
func AdvanceBody(id, seq uint16) {
  sim.AdvanceBody(id, seq)
}

// export GetNextAngle
func GetNextAngle(id uint16) int32 {
  cb := sim.GetControlledBody(id)
  if cb != nil {
    return cb.GetBody().NextAngle
  }

  return int32(0)
}

// export GetNextPosition
func GetNextPosition(id uint16) []float32 {
  b := sim.GetBody(id)
  if b != nil {
    return []float32{b.NextPos.X.Float(), b.NextPos.Y.Float(), b.NextPos.Z.Float()}
  }

  return []float32{0, 0, 0}
}

func main(){}
