package spacesim

import(
  "log"
  "github.com/ezmicken/fixpoint"
)

type ControlledBody struct {
  inputSeq          uint16

  rotationSpeed     int32
  thrust            fixpoint.Q16
  sqrMaxSpeed       fixpoint.Q16
  maxSpeed          fixpoint.Q16

  stateBuffer       *StateBuffer
  body              *Body

  lastInputSeq      uint16
}

var maxBlocks int = 256

// instantiation
///////////////////////

func NewControlledBody(r int32, t, s, blockScale fixpoint.Q16, bodyInfo BodyInfo) (*ControlledBody) {
  var cbod ControlledBody
  cbod.body = NewBody(bodyInfo, blockScale)
  cbod.stateBuffer = NewStateBuffer(256)
  cbod.rotationSpeed = r
  cbod.thrust = t
  cbod.maxSpeed = s
  cbod.sqrMaxSpeed = s.Mul(s)
  cbod.lastInputSeq = 0

  return &cbod
}

func (cb *ControlledBody) Initialize(ht HistoricalTransform) {
  cb.body.Initialize(ht)
  cb.stateBuffer.Initialize(ht)
  cb.inputSeq = ht.Seq
}

// Reliable ordered UDP ensures that input is pushed in order.
// Redundant pushes will silently fail.
func (cb *ControlledBody) PushInput(seq uint16, input byte) {
  if seq > cb.lastInputSeq {
    cb.stateBuffer.PushInput(seq, input)
    cb.lastInputSeq = seq
  } else {
    //log.Printf("Redundant input @ %v", seq)
  }
}

func (cb *ControlledBody) InputToState(ht HistoricalTransform, moveshoot byte) HistoricalTransform {
  acceleration := fixpoint.ZeroQ16

  left := BitOn(moveshoot, 0)
  right := BitOn(moveshoot, 1)
  forward := BitOn(moveshoot, 2)
  backward := BitOn(moveshoot, 3)

  if left && !right {
    ht.AngleDelta = cb.rotationSpeed
  } else if !left && right {
    ht.AngleDelta = -cb.rotationSpeed
  } else {
    ht.AngleDelta = 0
  }

  if forward && !backward {
    acceleration = cb.thrust
  } else if !forward && backward {
    acceleration = cb.thrust.Neg()
  } else {
    acceleration = fixpoint.ZeroQ16;
  }

  ht.Angle = WrapAngle(ht.Angle + ht.AngleDelta)

  if acceleration != fixpoint.ZeroQ16 {
    accVec := LUTAccel[ht.Angle].Mul(acceleration)
    originalVel := ht.Velocity
    ht.Velocity = ht.Velocity.Add(accVec)

    sqrX := ht.Velocity.X.Mul(ht.Velocity.X)
    sqrY := ht.Velocity.Y.Mul(ht.Velocity.Y)
    sqrMagnitude := sqrX.Add(sqrY)

    if sqrMagnitude.N > cb.sqrMaxSpeed.N {
      ht.Velocity = ht.Velocity.Normalize().Mul(cb.maxSpeed)
    }
    ht.VelocityDelta = ht.Velocity.Sub(originalVel)
  } else {
    ht.VelocityDelta = fixpoint.ZeroVec3Q16
  }

  return ht
}

func (cb *ControlledBody) Advance(seq uint16) {
  // Set the position based on last frame's state
  ht := cb.body.Move(seq)

  // get input from buffer
  input := cb.stateBuffer.GetNextInput()

  if input.Seq != ((int)(ht.Seq)) {
    log.Printf("%v %v", input.Seq, ht.Seq)
    panic("input seq did not match")
  }

  // apply input to body
  ht = cb.InputToState(ht, input.Data)

  // update collider based on new state and detect collision
  ht = cb.body.Collide(ht)

  cb.body.Commit(ht)
  cb.stateBuffer.PushState(ht)
}

func (cb *ControlledBody) GetAngle(seq uint16) int32 {
  ht := cb.stateBuffer.Get(seq)
  return ht.Angle
}

func (cb *ControlledBody) GetPositionX(seq uint16) fixpoint.Q16 {
  ht := cb.stateBuffer.Get(seq)
  return ht.Position.X
}

func (cb *ControlledBody) GetPositionY(seq uint16) fixpoint.Q16 {
  ht := cb.stateBuffer.Get(seq)
  return ht.Position.Y
}

func (cb *ControlledBody) GetVelocityX(seq uint16) fixpoint.Q16 {
  ht := cb.stateBuffer.Get(seq)
  return ht.Velocity.X
}

func (cb *ControlledBody) GetVelocityY(seq uint16) fixpoint.Q16 {
  ht := cb.stateBuffer.Get(seq)
  return ht.Velocity.Y
}

func (cb *ControlledBody) AddBlock(x, y int32) {
  cb.body.AddBlock(x, y)
}

func (cb *ControlledBody) OverwriteState(ht HistoricalTransform) {
  cb.stateBuffer.OverwriteState(ht)
}

func (cb *ControlledBody) PeekState() HistoricalTransform {
  return cb.stateBuffer.PeekState()
}

func (cb *ControlledBody) Rewind(seq uint16) {
  cb.stateBuffer.Rewind(seq)
}

func (cb *ControlledBody) ClearInput() {
  cb.stateBuffer.ClearInput()
}

func (cb *ControlledBody) GetBody() *Body {
  return cb.body
}

func (cb *ControlledBody) SerializeState(data []byte, head int) int {
  return cb.stateBuffer.Serialize(data, head)
}
