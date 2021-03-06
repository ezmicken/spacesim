package spacesim

import(
  "testing"
)

func TestInitialize(t *testing.T) {
  ht := randomHT

  sb := NewStateBuffer(256)
  sb.Initialize(ht)

  if sb.currentSeq != 500 {
    t.Logf("%v was not 500", sb.currentSeq)
    t.Fail()
  }

  if sb.past[sb.pastHead].Seq != uint16(499) {
    t.Logf("%v was not 499", sb.past[sb.pastHead].Seq)
    t.Fail()
  }

  expectedVal := uint16(498)
  for i := sb.size-1; i > 0; i-- {
    if sb.past[i].Seq != expectedVal {
      t.Logf("%v != %v", sb.past[i].Seq, expectedVal)
      t.Fail()
    }
    expectedVal--
  }
}

func TestGetNextInput(t *testing.T) {
  ht := randomHT

  sb := NewStateBuffer(256)
  sb.Initialize(ht)

  sb.PushInput(uint16(501), byte(1))

  i := sb.GetNextInput()
  i = sb.GetNextInput()
  if i.Seq != 501 {
    t.Logf("%v is not %v", i.Seq, 501)
    t.Fail()
  }
}
