package main

import(
  "log"
)

type StateBuffer struct {
  past        []HistoricalTransform
  future      []HistoricalTransform
  current     HistoricalTransform

  size        int
  currentSeq  int
  dirtySeq    int
  futureHead  int
  pastHead    int
}

func NewStateBuffer(size int) *StateBuffer {
  var sb StateBuffer
  sb.past = make([]HistoricalTransform, size)
  sb.future = make([]HistoricalTransform, size)
  sb.size = size
  sb.currentSeq = 0
  sb.futureHead = 0
  sb.pastHead = 0
  sb.dirtySeq = -1

  return &sb
}

func (sb *StateBuffer) Initialize(ht HistoricalTransform) {
  log.Printf("Initializing statebuffer @ %d", ht.Seq)
  s := int(ht.Seq)
  sb.currentSeq = s
  sb.current = ht
  sb.current.Seq = ht.Seq

  pt := ht
  pt.Seq = ht.Seq - uint16(sb.size)
  sb.past[sb.pastHead] = pt

  ft := ht
  ft.Seq = ht.Seq + uint16(sb.size)
  sb.future[sb.futureHead] = ft

  for i := 0; i < sb.size - 1; i++ {
    pt.Seq++
    ft.Seq--;
    sb.pastHead = sb.wrap(sb.pastHead + 1)
    sb.futureHead = sb.wrap(sb.futureHead + 1)
    sb.past[sb.pastHead] = pt
    sb.future[sb.futureHead] = ft
  }
}

func (sb *StateBuffer) Get(seq uint16) HistoricalTransform {
  s := int(seq)
  if s == sb.currentSeq {
    return sb.current
  }

  if s > sb.currentSeq {
    return sb.future[sb.wrap(sb.futureHead + 1 - (s - sb.currentSeq))]
  }

  if s < sb.currentSeq {
    return sb.past[sb.wrap(sb.pastHead + 1 - (sb.currentSeq - s))]
  }

  panic("bad historical transform")
}

func (sb *StateBuffer) Advance() HistoricalTransform {
  result := sb.current

  sb.pastHead = sb.wrap(sb.pastHead + 1)
  sb.past[sb.pastHead] = sb.current

  sb.current = sb.future[sb.futureHead]

  ft := sb.future[sb.wrap(sb.futureHead + 1)]
  ft.Seq++
  ft.Position = ft.Position.Add(ft.Velocity)
  sb.future[sb.futureHead] = ft

  sb.futureHead = sb.wrap(sb.futureHead - 1)
  sb.currentSeq++

  return result
}

func (sb *StateBuffer) Insert(ht HistoricalTransform, offset int) {
  s := int(ht.Seq)
  if s == sb.currentSeq && ht != sb.current {
    log.Printf("Cutting it close! %v", s)
    sb.dirty(sb.currentSeq + offset)
    sb.current = ht
  } else if s < sb.currentSeq {
    //log.Printf("Missed by %v", sb.currentSeq - ht.Seq)
    idx := sb.wrap(sb.pastHead - (sb.currentSeq - s))

    if sb.past[idx].Seq != ht.Seq {
      idx = sb.wrap(idx - int(sb.past[idx].Seq - ht.Seq))
    }

    if ht != sb.past[idx] {
      sb.dirty(s + offset)
      sb.past[idx] = ht
    }
  } else if s > sb.currentSeq {
    //log.Printf("hit! ahead by %v", ht.Seq - sb.currentSeq)
    diff := s - sb.currentSeq
    idx := sb.futureHead
    if diff > 1 {
      idx = sb.wrap(sb.futureHead + 1 - diff)
    }

    if ht != sb.future[idx] {
      sb.dirty(s + offset)
      sb.future[idx] = ht
    }
  }
}

func (sb *StateBuffer) Clean() {
  if sb.dirtySeq == -1 {
    return
  }

  if sb.dirtySeq < sb.currentSeq {
    diff := sb.currentSeq - sb.dirtySeq
    cleanIdx := sb.wrap(sb.pastHead - diff)
    dirtyIdx := sb.wrap(cleanIdx + 1)
    for i := 0; i < diff; i++ {
      sb.past[dirtyIdx].Angle = WrapAngle(sb.past[cleanIdx].Angle + sb.past[dirtyIdx].AngleDelta)
      sb.past[dirtyIdx].Position = sb.past[cleanIdx].Position.Add(sb.past[cleanIdx].Velocity)
      sb.past[dirtyIdx].Velocity = sb.past[cleanIdx].Velocity.Add(sb.past[dirtyIdx].VelocityDelta)
      cleanIdx = dirtyIdx
      dirtyIdx = sb.wrap(cleanIdx + 1)
      sb.dirtySeq++
    }
  }

  if sb.dirtySeq == sb.currentSeq {
    sb.current.Angle = WrapAngle(sb.past[sb.pastHead].Angle + sb.current.AngleDelta)
    sb.current.Position = sb.past[sb.pastHead].Position.Add(sb.past[sb.pastHead].Velocity)
    sb.current.Velocity = sb.past[sb.pastHead].Velocity.Add(sb.current.VelocityDelta)
    sb.dirtySeq++
  }

  if sb.dirtySeq == sb.currentSeq + 1 {
    sb.future[sb.futureHead].Angle = WrapAngle(sb.current.Angle + sb.future[sb.futureHead].AngleDelta)
    sb.future[sb.futureHead].Position = sb.current.Position.Add(sb.current.Velocity)
    sb.future[sb.futureHead].Velocity = sb.current.Velocity.Add(sb.future[sb.futureHead].VelocityDelta)
    sb.dirtySeq++
  }

  if sb.dirtySeq > sb.currentSeq {
    cleanIdx := sb.futureHead
    dirtyIdx := sb.wrap(cleanIdx - 1)

    for i := 0; i < sb.size - 1; i++ {
      sb.future[dirtyIdx].Angle = WrapAngle(sb.future[cleanIdx].Angle + sb.future[dirtyIdx].AngleDelta)
      sb.future[dirtyIdx].Position = sb.future[cleanIdx].Position.Add(sb.future[cleanIdx].Velocity)
      sb.future[dirtyIdx].Velocity = sb.future[cleanIdx].Velocity.Add(sb.future[dirtyIdx].VelocityDelta)
      cleanIdx = dirtyIdx
      dirtyIdx = sb.wrap(cleanIdx - 1)
    }
  }

  sb.dirtySeq = -1
}

func (sb *StateBuffer) GetCurrentSeq() uint16 {
  return uint16(sb.currentSeq)
}

func (sb *StateBuffer) dirty(seq int) {
  if seq < sb.dirtySeq || sb.dirtySeq == -1 {
    sb.dirtySeq = seq
  }
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
