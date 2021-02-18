package spacesim

import(
  "encoding/binary"
  "log"
)

type StateBuffer struct {
  past        []HistoricalTransform
  future      []Input

  size        int

  currentSeq  int
  pastHead    int // could be anything, likely to be close to currentSeq
  futureHead  int // should be currentSeq
}

type Input struct {
  Seq int
  Data byte
}

func NewStateBuffer(size int) *StateBuffer {
  var sb StateBuffer
  sb.past = make([]HistoricalTransform, size)
  sb.future = make([]Input, size)
  sb.size = size
  sb.currentSeq = 0
  sb.futureHead = 0
  sb.pastHead = 0

  return &sb
}

func (sb *StateBuffer) Initialize(ht HistoricalTransform) {
  log.Printf("Initializing statebuffer @ %v", ht.Seq)
  sb.currentSeq = int(ht.Seq)

  i := sb.pastHead;

  for {
    ht.Seq--
    if sb.past[i].Seq == uint16(sb.currentSeq-1) {
      break
    }
    sb.past[i] = ht
    i = sb.wrap(i-1)
  }
}

func (sb *StateBuffer) PushInput(seq uint16, data byte) {
  s := int(seq)
  var in Input
  in.Seq = s
  in.Data = data

  if s < sb.currentSeq {
    // TODO: handle the case where input is in the past -- rollback
    log.Printf("Input %v is in the past!", in)
  }
  idx := sb.wrap(sb.futureHead + s - sb.currentSeq)

  sb.future[idx] = in
}

func (sb *StateBuffer) Rewind(seq uint16) {
  if int(seq) > sb.currentSeq {
    log.Printf("Cannot rewind to the future.")
    return
  }
  if int(seq) == sb.currentSeq  {
    return
  }

  sb.futureHead = sb.wrap(sb.futureHead - (sb.currentSeq - int(seq)))
  sb.pastHead = sb.wrap(sb.pastHead - (int(sb.past[sb.pastHead].Seq) - int(seq)) - 1)
  sb.currentSeq = int(seq)
}

func (sb *StateBuffer) GetNextInput() Input {
  result := sb.future[sb.futureHead]

  if result.Seq != sb.currentSeq {
    result = Input{sb.currentSeq, byte(0)}
    sb.future[sb.futureHead] = result
  }

  sb.futureHead = sb.wrap(sb.futureHead + 1)
  sb.currentSeq++

  return result
}

func (sb *StateBuffer) PushState(ht HistoricalTransform) {
  current := sb.past[sb.pastHead].Seq
  rolledOver := (current == 65535 && ht.Seq - 1 == 0)
  if ht.Seq - 1 != current && !rolledOver {
    log.Printf("Pushed state is in the future %v -- %v", sb.past[sb.pastHead].Seq, ht.Seq - 1)
    panic("StateBuffer is out of sync!")
  }

  sb.pastHead = sb.wrap(sb.pastHead + 1)
  sb.past[sb.pastHead] = ht
}

func (sb *StateBuffer) OverwriteState(ht HistoricalTransform) {
  sb.past[sb.pastHead] = ht
}

func (sb *StateBuffer) PeekState() HistoricalTransform {
  return sb.past[sb.pastHead]
}

func (sb *StateBuffer) Get(seq uint16) HistoricalTransform {
  s := int(seq)

  if s < sb.currentSeq {
    return sb.past[sb.wrap(sb.pastHead + 1 - (sb.currentSeq - s))]
  }

  return sb.past[sb.pastHead]
}

func (sb *StateBuffer) GetCurrentSeq() uint16 {
  return uint16(sb.currentSeq)
}

func (sb *StateBuffer) Serialize(data []byte, head int) int {
  ht := sb.past[sb.pastHead]
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

  // make space for input count
  inputCountIdx := head
  head += 1

  seq := int(ht.Seq)
  i := sb.futureHead
  count := 0
  for {
    if sb.future[i].Seq != seq + count + 1 {
      break
    }
    data[head] = sb.future[i].Data
    head++
    count++
    i = sb.wrap(i + 1)
  }

  data[inputCountIdx] = byte(count)
  return head
}

func (sb *StateBuffer) wrap(idx int) int {
  for idx >= sb.size {
    idx -= sb.size
  }
  for idx < 0 {
    idx += sb.size
  }
  return idx
}
