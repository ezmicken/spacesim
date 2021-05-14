package spacesim

import(
  //"log"
  "encoding/binary"
  "github.com/ezmicken/fixpoint"
)

// Information about the body.
type BodyInfo struct {
  Id                uint16
  Size              int32
  Proximity         int32
  Lifetime          int32
  BounceCoefficient float32
  VelocityX         float32
  VelocityY         float32
}

// Describes a movement over the smallest amount of time.
type Movement struct {
  PositionX float32
  PositionY float32
  VelocityX float32
  VelocityY float32
  Time      float32
}

type Body struct {
  Angle             int32
  NextAngle         int32

  Pos               fixpoint.Vec3Q16
  NextPos           fixpoint.Vec3Q16
  Vel               fixpoint.Vec3Q16

  collider          *Collider
  blocks            []Rect
  blockHead         int
  blockTail         int
  blockSize         fixpoint.Q16
  maxBlocks         int

  history           []HistoricalTransform
  historyIdx        int

  movements         []Movement
  movementHead      int
  movementTail      int

  info              BodyInfo
  bounceCoefficient fixpoint.Q16

  firstFrame        bool
  dead              bool
}

var historyLength int = 1024
var movementLength int = 32

func NewBody(bodyInfo BodyInfo, blockSize fixpoint.Q16) *Body {
  var b Body
  b.history = make([]HistoricalTransform, historyLength)
  b.historyIdx = 0
  b.Angle = 0
  b.NextAngle = 0
  b.NextPos = fixpoint.ZeroVec3Q16
  b.Pos = fixpoint.ZeroVec3Q16
  b.Vel = fixpoint.ZeroVec3Q16
  b.firstFrame = true;
  b.dead = false
  b.blocks = make([]Rect, maxBlocks)
  b.blockHead = 0
  b.blockTail = 0
  b.collider = NewCollider(bodyInfo.Size)
  b.info = bodyInfo
  b.bounceCoefficient = fixpoint.Q16FromFloat(bodyInfo.BounceCoefficient)
  b.blockSize = blockSize
  b.movements = make([]Movement, movementLength)
  b.movementHead = 0
  b.movementTail = 0

  return &b
}

func (b *Body) Initialize(ht HistoricalTransform) {
  b.history[b.historyIdx] = ht
  b.Pos = ht.Position
  b.NextPos = ht.Position
  b.Vel = ht.Velocity
}

func (b *Body) Advance(seq uint16) {
  b.movementTail = b.movementHead
  var ht HistoricalTransform
  if b.firstFrame {
    b.firstFrame = false
    ht = b.history[b.historyIdx]
    ht = b.Collide(ht)
  } else {
    ht = b.Move(seq)
    ht = b.Collide(ht)
  }

  b.Commit(ht)
}

func (b *Body) Move(seq uint16) HistoricalTransform {
  b.Angle = b.NextAngle
  b.Pos = b.NextPos

  ht := b.history[b.historyIdx]
  b.historyIdx = wrapIdx(b.historyIdx + 1, historyLength)
  ht.Seq = seq

  return ht;
}

// update the collider and check for collision
func (b *Body) Collide(ht HistoricalTransform) HistoricalTransform {
  // grab a slice of relevant blocks
  var blockSlice []Rect
  if b.blockTail > b.blockHead {
    blockSlice = append(b.blocks[b.blockTail:], b.blocks[:b.blockHead]...)
  } else {
    blockSlice = b.blocks[b.blockTail:b.blockHead]
  }

  b.collider.Update(ht.Position, ht.Velocity)
  pos := ht.Position
  vel := ht.Velocity
  collision := b.collider.Check(pos, vel, blockSlice)
  remainingTime := fixpoint.OneQ16.Sub(collision.Time)
  if remainingTime.N <= fixpoint.ZeroQ16.N {
    pos.X = pos.X.Add(vel.X)
    pos.Y = pos.Y.Add(vel.Y)
    b.addMovement(pos, vel, fixpoint.OneQ16)
  } else {
    var accumulatedTime fixpoint.Q16 = fixpoint.ZeroQ16
    for cc := 1; remainingTime.N > fixpoint.ZeroQ16.N && cc <= 4; cc++ {
      pos = pos.Add(vel.Mul(collision.Time))

      // deflect unless slow enough to stop
      if fixpoint.Abs(collision.Normal.X).N > fixpoint.ZeroQ16.N {
        if fixpoint.Abs(vel.X).N < fixpoint.OneQ16.N {
          vel.X = fixpoint.ZeroQ16
        } else {
          vel.X = vel.X.Mul(b.bounceCoefficient).Neg()
        }
      } else if fixpoint.Abs(collision.Normal.Y).N > fixpoint.ZeroQ16.N {
        if fixpoint.Abs(vel.Y).N < fixpoint.OneQ16.N {
          vel.Y = fixpoint.ZeroQ16
        } else {
          vel.Y = vel.Y.Mul(b.bounceCoefficient).Neg()
        }
      }

      // Add a movement with this information.
      accumulatedTime = accumulatedTime.Add(collision.Time)
      b.addMovement(pos, vel, accumulatedTime)

      // Check for a new collision from this point.
      b.collider.Update(pos, vel)
      collision = b.collider.Check(pos, vel, blockSlice)
      if collision.Time.N >= remainingTime.N {
        pos = pos.Add(vel.Mul(remainingTime))
        b.addMovement(pos, vel, fixpoint.OneQ16)
      }

      remainingTime = remainingTime.Sub(collision.Time)
    }
  }

  ht.VelocityDelta = vel.Sub(ht.Velocity.Sub(ht.VelocityDelta))
  ht.Position = pos
  ht.Velocity = vel

  b.blockTail = b.blockHead

  return ht
}

// adopt and commit changes to history
func (b *Body) Commit(ht HistoricalTransform) {
  b.NextPos = ht.Position
  b.NextAngle = ht.Angle
  b.Vel = ht.Velocity
  b.history[b.historyIdx] = ht
}

func (b *Body) AddBlock(x, y int32) {
  fixedX := fixpoint.Q16FromInt32(x).Mul(b.blockSize)
  fixedY := fixpoint.Q16FromInt32(y).Mul(b.blockSize)

  b.blocks[b.blockHead] = NewRect(fixedX, fixedY, b.blockSize, b.blockSize)
  b.blockHead = wrapIdx(b.blockHead + 1, maxBlocks)
}

func (b *Body) OverwriteState(ht HistoricalTransform) {
  b.history[b.historyIdx] = ht
}

func (b *Body) Rewind(frames int) {
  b.historyIdx = wrapIdx(b.historyIdx - frames, historyLength)
}

func (b *Body) SerializeState(data []byte, head int) int {
  ht := b.history[b.historyIdx]
  binary.LittleEndian.PutUint16(data[head:head+2], uint16(ht.Angle))
  head += 2
  binary.LittleEndian.PutUint16(data[head:head+2], uint16(ht.AngleDelta))
  head += 2
  binary.LittleEndian.PutUint32(data[head:head+4], uint32(ht.Position.X.N))
  head += 4
  binary.LittleEndian.PutUint32(data[head:head+4], uint32(ht.Position.Y.N))
  head += 4
  binary.LittleEndian.PutUint32(data[head:head+4], uint32(ht.Velocity.X.N))
  head += 4
  binary.LittleEndian.PutUint32(data[head:head+4], uint32(ht.Velocity.Y.N))
  head += 4
  binary.LittleEndian.PutUint32(data[head:head+4], uint32(ht.VelocityDelta.X.N))
  head += 4
  binary.LittleEndian.PutUint32(data[head:head+4], uint32(ht.VelocityDelta.Y.N))
  head += 4
  return head
}

func (b *Body) Kill() {
  b.dead = true
}

func (b *Body) addMovement(pos, vel fixpoint.Vec3Q16, time fixpoint.Q16) {
  b.movements[b.movementHead] = Movement {
    pos.X.Float(),
    pos.Y.Float(),
    vel.X.Float(),
    vel.Y.Float(),
    time.Float(),
  }
  b.movementHead = wrapIdx(b.movementHead+1, movementLength)
}
func (b *Body) GetMovement() Movement {
  m := b.movements[b.movementTail]
  limit := wrapIdx(b.movementHead-1, movementLength)
  if (b.movementTail != limit) {
    b.movementTail = wrapIdx(b.movementTail+1, movementLength)
  }
  return m
}

func (b *Body) IsDead()     bool  { return b.dead }

func wrapIdx(idx, max int) int {
  for idx >= max { idx -= max }
  for idx < 0 { idx += max }
  return idx
}
