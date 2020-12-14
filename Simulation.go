package spacesim

import (
  "sync"
  "github.com/ezmicken/fixpoint"
)

type Simulation struct {
  controlledBodies    sync.Map
  bodiesById          sync.Map
  allBodies           []*Body
  scale               fixpoint.Q16
  halfScale           fixpoint.Q16

  seq                 uint16
  timestep            fixpoint.Q16
}

var rotationSpeed int32 = 9
var thrust fixpoint.Q16 = fixpoint.Q16FromFloat(float32(0.36))
var maxSpeed fixpoint.Q16 = fixpoint.Q16FromFloat(float32(20))

func NewSimulation(ts, scale fixpoint.Q16) *Simulation {
  var s Simulation
  s.timestep = ts
  s.scale = scale
  s.halfScale = scale.Div(fixpoint.Q16FromInt32(2))
  s.seq = 0
  s.allBodies = []*Body{}
  return &s
}

func (s *Simulation) GetControlledBody(id uint16) *ControlledBody {
  cb, ok := s.controlledBodies.Load(id)
  if ok {
    return cb.(*ControlledBody)
  } else {
    return nil
  }
}

func (s *Simulation) GetBody(id uint16) *Body {
  b, ok := s.bodiesById.Load(id)
  if ok {
    return b.(*Body)
  } else {
    return nil
  }
}

func (s *Simulation) AddControlledBody(id uint16, x, y, d int32) {
  cb := NewControlledBody(rotationSpeed, d, thrust, maxSpeed, s)
  s.allBodies = append(s.allBodies, cb.GetBody())
  s.controlledBodies.Store(id, cb)
  s.bodiesById.Store(id, cb.GetBody())

  xPos := fixpoint.Q16FromInt32(x).Mul(s.scale).Add(s.halfScale)
  yPos := fixpoint.Q16FromInt32(y).Mul(s.scale).Add(s.halfScale)

  var ht HistoricalTransform
  ht.Seq = s.seq
  ht.Angle = 0
  ht.AngleDelta = 0
  ht.Position = fixpoint.Vec3Q16{xPos, yPos, fixpoint.ZeroQ16}
  ht.Velocity = fixpoint.ZeroVec3Q16
  ht.VelocityDelta = fixpoint.ZeroVec3Q16

  cb.Initialize(ht)

  return
}

func (s *Simulation) RemoveControlledBody(id uint16) {
  cb, ok := s.controlledBodies.Load(id)
  if ok && cb != nil {
    cb.(*ControlledBody).GetBody().Kill()
    s.controlledBodies.Delete(id)
    s.bodiesById.Delete(id)
  }
}

func (s *Simulation) AdvanceControlledBody(id, seq uint16) {
  cb := s.GetControlledBody(id)
  if cb != nil {
    cb.Advance(seq)
  }
}
func (s *Simulation) AdvanceBody(id, seq uint16) {
  b := s.GetBody(id)
  if b != nil {
    b.Advance(seq)
  }
}

func (s *Simulation) Advance(seq int) {
  s.seq++
  // Update all bodies
  // flag dead bodies for removal
  // process live bodies
  // replace ps.bodies with filtered list
  filteredBodies := s.allBodies[:0]
  for i, b := range s.allBodies {
    if !b.IsDead() {
      filteredBodies = append(filteredBodies, b)
    } else {
      s.allBodies[i] = nil
      continue
    }
  }
  s.allBodies = filteredBodies
}
