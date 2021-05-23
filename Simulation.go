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

func (s *Simulation) ControlBody(id uint16, rotationSpeed int32, thrust, maxSpeed float32) {
  b, ok := s.bodiesById.Load(id)
  if ok && b != nil {
    thrustQ16 := fixpoint.Q16FromFloat(thrust)
    maxSpeedQ16 := fixpoint.Q16FromFloat(maxSpeed)
    cb := NewControlledBody(rotationSpeed, thrustQ16, maxSpeedQ16, s.scale, b.(*Body))
    s.controlledBodies.Store(id, cb)
  }
}

func (s *Simulation) RemoveControlledBody(id uint16) {
  cb, ok := s.controlledBodies.Load(id)
  if ok && cb != nil {
    cb.(*ControlledBody).GetBody().Kill()
    s.controlledBodies.Delete(id)
    s.bodiesById.Delete(id)
  }
}

func (s *Simulation) AddBody(id uint16, x, y float32, bodyInfo BodyInfo) {
  xPos := fixpoint.Q16FromFloat(x)
  yPos := fixpoint.Q16FromFloat(y)
  xVel := fixpoint.Q16FromFloat(bodyInfo.VelocityX)
  yVel := fixpoint.Q16FromFloat(bodyInfo.VelocityY)

  body := NewBody(bodyInfo, s.scale)
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

func (s *Simulation) AddSerializedBody(sb SerializedBody) {
  body := NewBodyFromSerialized(sb, s.scale)
  s.allBodies = append(s.allBodies, body)
  s.bodiesById.Store(sb.Id, body)
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
  frames := int(s.seq - seq)
  s.controlledBodies.Range(func(key, value interface{}) bool {
    b := value.(*ControlledBody)
    b.Rewind(frames)
    return true
  })
  allBodiesLen := len(s.allBodies)
  for i := 0; i < allBodiesLen; i++ {
    s.allBodies[i].Rewind(frames)
  }
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
// Body count           | uint16 |
// Body list              -----
//   - id               | uint16 |
//   - owner            | uint16 |
//   - size             | byte   |
//   - proximity        | byte   |
//   - bounceCoeff      | int32  | (fixpoint float)
//   - angle            | uint16 |
//   - delta angle      | uint16 |
//   - position X       | int32  | (fixpoint float)
//   - position Y       | int32  | (fixpoint float)
//   - velocity X       | int32  | (fixpoint float)
//   - velocity Y       | int32  | (fixpoint float)
//   - delta velocity X | int32  | (fixpoint float)
//   - delta velocity Y | int32  | (fixpoint float)
//   ...
// Players count        | byte   |
// Players list           ------
//   - body id          | uint16 |
//   - input count      | byte   |
//   - input            | byte   |
//   ...
// 5 + 1 + (players * (3 + inputCount)) + 2 + (30*bodies)
func (s *Simulation) SerializeState(data []byte, head int) int {
  dataSizeIdx := head
  head += 2
  bodyCount := len(s.allBodies)
  binary.LittleEndian.PutUint16(data[head:head+2], uint16(bodyCount))
  head += 2
  for i := 0; i < bodyCount; i++ {
    head = s.allBodies[i].SerializeState(data, head)
  }
  cbCountIdx := head
  head += 1
  cbCount := 0
  s.controlledBodies.Range(func(key, value interface{}) bool {
    binary.LittleEndian.PutUint16(data[head:head+2], key.(uint16))
    head += 2
    cb := value.(*ControlledBody)
    head = cb.SerializeState(data, head)
    cbCount++
    return true
  })
  data[cbCountIdx] = byte(cbCount)

  binary.LittleEndian.PutUint16(data[dataSizeIdx:dataSizeIdx+2], uint16(head - dataSizeIdx))

  return head
}
