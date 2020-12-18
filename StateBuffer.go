package spacesim

import(
  "log"
)

type StateBuffer struct {
  past        []HistoricalTransform
  future      []Input

  size        int
  currentSeq  int

  pastHead    int
  futureHead  int
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

func (sb *StateBuffer) GetNextInput() Input {
  result := sb.future[sb.futureHead]
  if result.Seq != sb.currentSeq {
    result = Input{sb.currentSeq, byte(0)}
  }

  sb.futureHead = wrap(sb.futureHead + 1)
  sb.currentSeq++

  return result
}

func (sb *StateBuffer) PushState(ht HistoricalTransform) {
  if ht.Seq != uint16(sb.currentSeq) {
    log.Printf("Pushed state is in the future %v -- %v", sb.past[sb.pastHead].Seq, ht.Seq - 1)
    panic("StateBuffer is out of sync!")
  }

  sb.pastHead = sb.wrap(sb.pastHead + 1)
  sb.past[sb.pastHead] = ht
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

func (sb *StateBuffer) wrap(idx int) int {
  for idx >= sb.size {
    idx -= sb.size
  }
  for idx < 0 {
    idx += sb.size
  }
  return idx
}
