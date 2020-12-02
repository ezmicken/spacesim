package main

import(
  //"log"
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
}


// instantiation
///////////////////////

func NewControlledBody(r int32, t, s fixpoint.Q16, sim *Simulation) (*ControlledBody) {
  var cbod ControlledBody
  cbod.stateBuffer = NewStateBuffer(256)
  cbod.rotationSpeed = r
  cbod.thrust = t
  cbod.maxSpeed = s
  cbod.sqrMaxSpeed = s.Mul(s)
  cbod.body = NewBody()
//  cbod.Collider = NewSpaceCollider(96, 48)

  return &cbod
}

func (cb *ControlledBody) Initialize(ht HistoricalTransform) {
  cb.stateBuffer.Initialize(ht)
  cb.inputSeq = ht.Seq
  cb.body.Pos = ht.Position
  cb.body.Vel = ht.Velocity
}

func (cb *ControlledBody) InputToState(seq uint16, moveshoot byte) {
  if seq < cb.inputSeq {
    return
  } else {
    cb.inputSeq = seq
  }

  ht := cb.stateBuffer.Get(seq - 1)

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

  ht.Seq++
  cb.stateBuffer.Insert(ht, 0)
  cb.stateBuffer.Clean()
}

func (cb *ControlledBody) Advance(seq uint16) {
  cb.body.Advance(seq)

  if cb.stateBuffer.GetCurrentSeq() <= (seq - 1) {
    ht := cb.stateBuffer.Advance()

    for ht.Seq < (seq - 1) {
      ht = cb.stateBuffer.Advance()
    }

    // cb.Collider.Update(ht.Position, ht.Velocity)
    // potentialCollisions := wm.GetBlockRects(cb.Collider.Broad)
    // if potentialCollisions != nil {
    //   cc := 1
    //   check2 := ht
    //   check, _ := cb.Collider.Check(ht, potentialCollisions)
    //   for check != check2 && cc <= 4 {
    //     check2 = check
    //     check, _ = cb.Collider.Check(check, potentialCollisions)
    //     cc++
    //   }

    //   if ht != check {
    //     stats := cb.controllingPlayer.Stats
    //     if check.Velocity.LenSqr() > (stats.MaxSpeed * stats.MaxSpeed) {
    //       check.Velocity = check.Velocity.Normalize().Mul(stats.MaxSpeed)
    //     }
    //     cb.stateBuffer.Insert(check, 1)
    //     cb.stateBuffer.Clean()
    //     ht = check
    //   }
    // }

    cb.body.NextPos = ht.Position
    cb.body.NextAngle = ht.Angle
    cb.body.Vel = ht.Velocity
  }

  return
}

func (cb *ControlledBody) GetBody() *Body {
  return cb.body
}
