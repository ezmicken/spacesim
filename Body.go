package spacesim

import(
  "encoding/binary"
  "github.com/ezmicken/fixpoint"
)

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

  firstFrame        bool
  dead              bool
}

var historyLength int = 1024

func NewBody(broadSize, narrowSize int32, blockSize fixpoint.Q16) *Body {
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
  b.collider = NewCollider(broadSize, narrowSize)
  b.blockSize = blockSize
  b.maxBlocks = 128

  return &b
}

func (b *Body) Initialize(ht HistoricalTransform) {
  b.history[b.historyIdx] = ht
  b.Pos = ht.Position
  b.NextPos = ht.Position
  b.Vel = ht.Velocity
}

func (b *Body) Advance(seq uint16) {
  var ht HistoricalTransform
  if b.firstFrame {
    b.firstFrame = false
    ht = b.history[b.historyIdx]
    ht = b.Collide(ht)
  } else {
    ht = b.Move(seq)
  }

  ht = b.Collide(ht)

  b.Commit(ht)
}

func (b *Body) Move(seq uint16) HistoricalTransform {
  b.Angle = b.NextAngle
  b.Pos = b.NextPos

  ht := b.history[b.historyIdx]
  b.historyIdx = wrapIdx(b.historyIdx + 1, historyLength)
  ht.Seq = seq

  ht.Position = ht.Position.Add(ht.Velocity)
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
  cc := 1
  check2 := ht
  check := b.collider.Check(ht, blockSlice)
  for check != check2 && cc <= 4 {
    // escape as soon as no collision is detected
    if check2 == check { break }

    check2 = check
    b.collider.Update(check.Position, check.Velocity)
    check = b.collider.Check(check, blockSlice)
    cc++
  }

  b.blockTail = b.blockHead

  ht = check

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
  b.blockHead = wrapIdx(b.blockHead + 1, b.maxBlocks)
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

func(b *Body) IsDead() bool {
  return b.dead
}

func (b *Body) Kill() {
  b.dead = true
}

func wrapIdx(idx, max int) int {
  for idx >= max { idx -= max }
  for idx < 0 { idx += max }
  return idx
}
