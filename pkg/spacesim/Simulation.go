package main

import (
  "sync"
  "github.com/ezmicken/fixpoint"
)

type Simulation struct {
  controlledBodies    sync.Map
  bodiesById          sync.Map
  allBodies           []*Body

  seq                 uint16
  timestep            fixpoint.Q16
}

var rotationSpeed int32 = 9
var thrust fixpoint.Q16 = fixpoint.Q16FromFloat(float32(0.36))
var maxSpeed fixpoint.Q16 = fixpoint.Q16FromFloat(float32(20))

func NewSimulation(ts fixpoint.Q16) *Simulation {
  var s Simulation
  s.timestep = ts
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

func (s *Simulation) AddControlledBody(id uint16) {
  cb := NewControlledBody(rotationSpeed, thrust, maxSpeed, s)
  s.allBodies = append(s.allBodies, cb.GetBody())
  s.controlledBodies.Store(id, cb)
  s.bodiesById.Store(id, cb.GetBody())
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
