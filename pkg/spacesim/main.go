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
func AddControlledBody() int32 {
  return sim.AddControlledBody()
}

// export RemoveControlledBody
func RemoveControlledBody(id int32) {
  sim.RemoveControlledBody(id)
}

// export PushInput
func PushInput(id int32, tick uint16, moveshoot byte) {
  cb := sim.GetControlledBody(id)
  if cb != nil {
    cb.InputToState(int(tick), moveshoot)
  }
}

// export Advance
func Advance(seq uint16) {
  sim.Advance(int(seq))
}

// export GetNextAngle
func GetNextAngle(id int32) int32 {
  cb := sim.GetControlledBody(id)
  if cb != nil {
    return cb.GetNextAngle()
  }

  return int32(0)
}

// export GetNextPosition
func GetNextPosition(id int32) []float32 {
  b := sim.GetBody(id)
  if b != nil {
    return []float32{b.NextPos.X.Float(), b.NextPos.Y.Float(), b.NextPos.Z.Float()}
  }

  return []float32{0, 0, 0}
}

func main(){}
