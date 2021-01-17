package spacesim

import(
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
  sim               *Simulation
  collider          *Collider

  blocks            []Rect
  blockHead         int

  delay             int
  previous          HistoricalTransform

  lastInputSeq      uint16
}

var maxBlocks int = 128

// instantiation
///////////////////////

func NewControlledBody(r, d int32, t, s fixpoint.Q16, sim *Simulation) (*ControlledBody) {
  var cbod ControlledBody
  cbod.stateBuffer = NewStateBuffer(256)
  cbod.rotationSpeed = r
  cbod.thrust = t
  cbod.maxSpeed = s
  cbod.sqrMaxSpeed = s.Mul(s)
  cbod.body = NewBody()
  cbod.collider = NewCollider(96, 48)
  cbod.blocks = make([]Rect, maxBlocks)
  cbod.blockHead = 0
  cbod.sim = sim
  cbod.delay = int(d)
  cbod.lastInputSeq = 0
  zero := fixpoint.ZeroVec3Q16
  cbod.previous = HistoricalTransform{0, 0, 0, zero, zero, zero}

  return &cbod
}

func (cb *ControlledBody) Initialize(ht HistoricalTransform) {
  cb.stateBuffer.Initialize(ht)
  cb.inputSeq = ht.Seq
  cb.body.Pos = ht.Position
  cb.body.Vel = ht.Velocity
  cb.previous = ht
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

func (cb *ControlledBody) InputToState(seq uint16, moveshoot byte) HistoricalTransform {
  ht := cb.previous

  acceleration := fixpoint.ZeroQ16

  ht.Position = ht.Position.Add(ht.Velocity)

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

  ht.Seq = seq

  return ht
}

func (cb *ControlledBody) Advance(seq uint16) {
  cb.body.Advance(seq)

  // get input from buffer
  input := cb.stateBuffer.GetNextInput()

  // apply input to body
  ht := cb.InputToState(uint16(input.Seq), input.Data)

  // update collider based on new state
  cb.collider.Update(ht.Position, ht.Velocity)

  // check for collision
  cc := 1
  check2 := ht
  check := cb.collider.Check(ht, cb.blocks)
  for check != check2 && cc <= 4 {
    check2 = check
    check = cb.collider.Check(check, cb.blocks)
    cc++
  }

  if ht != check {
    ht = check
  }
  cb.body.NextPos = ht.Position
  cb.body.NextAngle = ht.Angle
  cb.body.Vel = ht.Velocity

  // Commit to history.
  cb.stateBuffer.PushState(ht)
  cb.previous = ht

  return
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

func (cb *ControlledBody) AddBlock(x, y int32) {
  fixedX := fixpoint.Q16FromInt32(x).Mul(cb.sim.scale)
  fixedY := fixpoint.Q16FromInt32(y).Mul(cb.sim.scale)

  for i := 0; i < maxBlocks; i++ {
    block := cb.blocks[wrap(cb.blockHead-i)]
    if block.Min.X.N == fixedX.N && block.Min.Y.N == fixedY.N {
      return
    }
  }

  cb.blocks[cb.blockHead] = NewRect(fixedX, fixedY, cb.sim.scale, cb.sim.scale)
  cb.blockHead++
  cb.blockHead = wrap(cb.blockHead)
}

func wrap(i int) int {
  for i >= maxBlocks { i -= maxBlocks }
  for i < 0 { i += maxBlocks }

  return i
}

func (cb *ControlledBody) GetBody() *Body {
  return cb.body
}
