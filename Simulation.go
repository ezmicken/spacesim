package spacesim

import (
  "sync"
  "log"
  "encoding/binary"
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

func (s *Simulation) Reset() {
  s.seq = 0
  s.allBodies = []*Body{}
  s.controlledBodies = sync.Map{}
  s.bodiesById = sync.Map{}
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
  cb := NewControlledBody(rotationSpeed, d, thrust, maxSpeed, s.scale)
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
}

func (s *Simulation) RemoveControlledBody(id uint16) {
  cb, ok := s.controlledBodies.Load(id)
  if ok && cb != nil {
    cb.(*ControlledBody).GetBody().Kill()
    s.controlledBodies.Delete(id)
    s.bodiesById.Delete(id)
  }
}

func (s *Simulation) AddBody(id uint16, x, y, vx, vy float32, size int32) {
  xPos := fixpoint.Q16FromFloat(x)
  yPos := fixpoint.Q16FromFloat(y)
  xVel := fixpoint.Q16FromFloat(vx)
  yVel := fixpoint.Q16FromFloat(vy)

  // broad size is magnitude * 3
  sqrMag := xVel.Mul(xVel).Add(yVel.Mul(yVel))
  broadSize := sqrMag.InvSqrt().Mul(sqrMag).Mul(fixpoint.Q16FromInt32(3))

  body := NewBody(int32(broadSize.Float()), size, s.scale)
  s.allBodies = append(s.allBodies, body)
  s.bodiesById.Store(id, body)

  var ht HistoricalTransform
  ht.Seq = s.seq
  ht.Angle = 0
  ht.AngleDelta = 0
  ht.Position = fixpoint.Vec3Q16{xPos, yPos, fixpoint.ZeroQ16}
  ht.Velocity = fixpoint.Vec3Q16{xVel, yVel, fixpoint.ZeroQ16}
  ht.VelocityDelta = fixpoint.ZeroVec3Q16

  body.Initialize(ht)
}

func (s *Simulation) RemoveBody(id uint16) {
  body, ok := s.bodiesById.Load(id)
  if ok && body != nil {
    body.(*Body).Kill()
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

func (s *Simulation) Rewind(seq uint16) {
  s.controlledBodies.Range(func(key, value interface{}) bool {
    b := value.(*ControlledBody)
    b.Rewind(seq)
    return true
  })
  // TODO: bodies
}

// TODO: handle bodies
func (s *Simulation) OverwriteState(seq, id, angle, angleDelta uint16, x, y, vx, vy, dvx, dvy int32) {
  cb := s.GetControlledBody(id)
  if cb == nil {
    log.Printf("Attempted to rewind body that doesn't exist.")
    return
  }

  var ht HistoricalTransform
  ht.Seq = s.seq
  ht.Angle = int32(angle)
  ht.AngleDelta = int32(angleDelta)
  ht.Position = fixpoint.Vec3Q16{fixpoint.Q16{x}, fixpoint.Q16{y}, fixpoint.ZeroQ16}
  ht.Velocity = fixpoint.Vec3Q16{fixpoint.Q16{vx}, fixpoint.Q16{vy}, fixpoint.ZeroQ16}
  ht.VelocityDelta = fixpoint.Vec3Q16{fixpoint.Q16{dvx}, fixpoint.Q16{dvy}, fixpoint.ZeroQ16}

  return
}

func (s *Simulation) PeekState(id uint16) HistoricalTransform {
  cb := s.GetControlledBody(id)
  if cb == nil {
    log.Printf("Attempted to peek state of body that doesn't exist.")
    return InvalidHistoricalTransform
  }

  return cb.PeekState()
}

// TODO: handle bodies
// - size               | uint16 |
// - seq                | uint16 |
// ControlledBody count | byte
// ControlledBody list  | ------
//   - id               | uint16 |
//   - angle            | uint16 |
//   - angle delta      | uint16 |
//   - position X       | int32  |
//   - position Y       | int32  |
//   - velocity X       | int32  |--- 31
//   - velocity Y       | int32  |
//   - delta velocity X | int32  |
//   - delta velocity Y | int32  |
//   - input count      | byte   |
//   Input list         | -----
//     - input          | byte
//     ...
// ...
// Body count           | uint16
// Body list            | -----
//   - id               | uint16 |
//   - position X       | int32  |
//   - position Y       | int32  |
//   - velocity X       | int32  |--- 26
//   - velocity Y       | int32  |
//   - delta velocity X | int32  |
//   - delta velocity Y | int32  |
// ...
func (s *Simulation) SerializeState(data []byte, head int) int {
  dataSizeIdx := head
  head += 2
  cbCountIdx := head
  head += 1
  cbCount := 0
  s.controlledBodies.Range(func(key, value interface{}) bool {
    binary.LittleEndian.PutUint16(data[head:head+2], key.(uint16))
    head += 2
    b := value.(*ControlledBody)
    head = b.SerializeState(data, head)
    cbCount++
    return true
  })
  data[cbCountIdx] = byte(cbCount)

  // TODO: dynamic bodies.
  data[head] = byte(0)
  head++
  binary.LittleEndian.PutUint16(data[dataSizeIdx:dataSizeIdx+2], uint16(head - dataSizeIdx + 2))

  return head
}
