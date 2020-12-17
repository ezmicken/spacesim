package spacesim

import(
  "log"
)

type StateBuffer struct {
  past        []HistoricalTransform
  future      []byte

  size        int
  currentSeq  int

  futureHead  int
  pastHead    int
}

func NewStateBuffer(size int) *StateBuffer {
  var sb StateBuffer
  sb.past = make([]HistoricalTransform, size)
  sb.future = make([]byte, size)
  sb.size = size
  sb.currentSeq = 0
  sb.futureHead = 0
  sb.pastHead = 0

  return &sb
}

func (sb *StateBuffer) Initialize(ht HistoricalTransform) {
  log.Printf("Initializing statebuffer @ %v", ht.Seq)
  s := int(ht.Seq)
  sb.currentSeq = s

  pt := ht
  sb.past[sb.pastHead] = pt

  for i := 0; i < sb.size; i++ {
    sb.pastHead = wrap(sb.pastHead - 1)
    sb.past[sb.pastHead] = pt
  }
}

func (sb *StateBuffer) PushInput(seq uint16, input byte) {
  idx := sb.wrap(sb.futureHead + int(seq) - sb.currentSeq)
  sb.future[idx] = input

  // TODO: handle the case where offset is >= size
}

func (sb *StateBuffer) GetNextInput() byte {
  result := sb.future[sb.futureHead]
  sb.futureHead = wrap(sb.futureHead + 1)

  sb.currentSeq++

  return result
}

func (sb *StateBuffer) PushState(ht HistoricalTransform) {
  if sb.past[sb.pastHead].Seq != ht.Seq - 1 {
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

  log.Printf("Attempted to get state from future %v -- %v", seq, sb.currentSeq)
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
