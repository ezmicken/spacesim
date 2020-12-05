package spacesim

import(
  //"log"
  "github.com/ezmicken/fixpoint"
)

func WrapQ16(val, min, length fixpoint.Q16) fixpoint.Q16 {
  for val.N >= length.N { val = val.Sub(length) }
  for val.N < min.N { val = val.Add(length) }
  return val;
}

func WrapInt32(val, min, length int32) int32 {
  for val >= length { val -= length }
  for val < min { val += length }
  return val
}

func WrapAngle(val int32) int32 {
  return WrapInt32(val, 0, 360)
}

func WrapAngleQ16(val fixpoint.Q16) fixpoint.Q16 {
  return WrapQ16(val, fixpoint.Q16FromFloat(float32(0)), fixpoint.Q16FromFloat(float32(360)))
}

func BitOn(b byte, pos int) bool {
  return (b & (1 << pos)) != 0;
}

