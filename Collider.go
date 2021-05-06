package spacesim

import(
  //"log"
  "github.com/ezmicken/fixpoint"
)

type Collider struct {
  size            fixpoint.Q16
  halfSize        fixpoint.Q16

  narrow           Rect
}

type Collision struct {
  Time    fixpoint.Q16
  Normal  fixpoint.Vec3Q16
  Block   Rect
  Overlap Rect
}

func NewCollider(size int32) *Collider {
  var c Collider
  c.size = fixpoint.Q16FromInt32(size)
  c.halfSize = c.size.Div(fixpoint.TwoQ16)

  return &c
}

func (c *Collider) Update(position, velocity fixpoint.Vec3Q16) {
  c.narrow = NewRect(position.X.Sub(c.halfSize), position.Y.Sub(c.halfSize), c.size, c.size)
}

func (c *Collider) Check(position, velocity fixpoint.Vec3Q16, potentialCollisions []Rect) Collision {
  var closest Collision
  closest.Time = fixpoint.OneQ16
  closest.Normal = fixpoint.ZeroVec3Q16
  closest.Overlap = InvalidRect

  broadX := c.narrow.Min.X
  if velocity.X.N <= 0 {
    broadX = c.narrow.Min.X.Add(velocity.X)
  }
  broadY := c.narrow.Min.Y
  if velocity.Y.N <= 0 {
    broadY = c.narrow.Min.Y.Add(velocity.Y)
  }
  broadW := c.narrow.W.Add(fixpoint.Abs(velocity.X))
  broadH := c.narrow.H.Add(fixpoint.Abs(velocity.Y))
  broad := NewRect(broadX, broadY, broadW, broadH)

  // Iterate each piece of static geometry looking for collision.
  for i := 0; i < len(potentialCollisions); i++ {
    overlap := RectOverlap(broad, potentialCollisions[i])
    if overlap != InvalidRect {
      col := c.sweep(velocity, potentialCollisions[i])
      valid := true

      // invalidate the collision if the face is not exposed.
      // TODO: make this configurable.
      if col.Time.N < fixpoint.OneQ16.N {
        if col.Normal.X != fixpoint.ZeroQ16 {
          oneAway := col.Block.Min.X.Add(col.Normal.X.Mul(col.Block.W)).N
          twoAway := col.Block.Min.X.Add(col.Normal.X.Mul(col.Block.W).Mul(fixpoint.TwoQ16)).N
          for j := 0; j < len(potentialCollisions); j++ {
            if potentialCollisions[j] != potentialCollisions[i] {
              xmin := potentialCollisions[j].Min.X.N
              if (xmin == oneAway || xmin == twoAway) && potentialCollisions[j].Min.Y == col.Block.Min.Y {
                valid = false
                break;
              }
            }
          }
        } else if col.Normal.Y != fixpoint.ZeroQ16 {
          oneAway := col.Block.Min.Y.Add(col.Normal.Y.Mul(col.Block.H)).N
          twoAway := col.Block.Min.Y.Add(col.Normal.Y.Mul(col.Block.H).Mul(fixpoint.TwoQ16)).N
          for j := 0; j < len(potentialCollisions); j++ {
            if potentialCollisions[j] != potentialCollisions[i] {
              ymin := potentialCollisions[j].Min.Y.N
              if (ymin == oneAway || ymin == twoAway) && potentialCollisions[j].Min.X == col.Block.Min.X {
                valid = false
                break;
              }
            }
          }
        }
      }
      //closest is the one with the lowest time to entry and/or greatest overlap.
      if valid && col.Time.N < closest.Time.N {
        closest = col
      }
    }
  }

  return closest
}

func (c *Collider) sweep(velocity fixpoint.Vec3Q16, block Rect) Collision {
  var dxEntry fixpoint.Q16
  var dxExit fixpoint.Q16
  var dyEntry fixpoint.Q16
  var dyExit fixpoint.Q16
  var result Collision
  result.Block = block
  result.Time = fixpoint.OneQ16
  result.Normal = fixpoint.ZeroVec3Q16
  result.Overlap = RectOverlap(c.narrow, block)

  // handle the case where it will collide at some point
  // during this frame.
  if velocity.X.N > fixpoint.ZeroQ16.N {
    dxEntry = block.Min.X.Sub(c.narrow.Max.X)
    dxExit = block.Max.X.Sub(c.narrow.Min.X)
  } else {
    dxEntry = block.Max.X.Sub(c.narrow.Min.X)
    dxExit = block.Min.X.Sub(c.narrow.Max.X)
  }

  if velocity.Y.N > fixpoint.ZeroQ16.N {
    dyEntry = block.Min.Y.Sub(c.narrow.Max.Y)
    dyExit = block.Max.Y.Sub(c.narrow.Min.Y)
  } else {
    dyEntry = block.Max.Y.Sub(c.narrow.Min.Y)
    dyExit = block.Min.Y.Sub(c.narrow.Max.Y)
  }

  var txEntry fixpoint.Q16
  var txExit fixpoint.Q16
  var tyEntry fixpoint.Q16
  var tyExit fixpoint.Q16

  if velocity.X.N == fixpoint.ZeroQ16.N {
    txEntry = fixpoint.MaxQ16.Neg()
    txExit = fixpoint.MaxQ16
  } else {
    txEntry = dxEntry.Div(velocity.X)
    txExit = dxExit.Div(velocity.X)
  }

  if velocity.Y.N == fixpoint.ZeroQ16.N {
    tyEntry = fixpoint.MaxQ16.Neg()
    tyExit = fixpoint.MaxQ16
  } else {
    tyEntry = dyEntry.Div(velocity.Y)
    tyExit = dyExit.Div(velocity.Y)
  }

  entryTime := fixpoint.Max(txEntry, tyEntry)
  exitTime := fixpoint.Min(txExit, tyExit)

  exiting := entryTime.N > exitTime.N
  negativeEntry := txEntry.N < fixpoint.ZeroQ16.N && tyEntry.N < fixpoint.ZeroQ16.N
  futureEntry := txEntry.N > fixpoint.OneQ16.N || tyEntry.N > fixpoint.OneQ16.N

  if !exiting && !negativeEntry && !futureEntry {
    result.Time = entryTime
    if txEntry.N > tyEntry.N {
      if velocity.X.N < 0 {
        result.Normal = fixpoint.Vec3Q16FromFloat(1.0, 0.0, 0.0)
      } else {
        result.Normal = fixpoint.Vec3Q16FromFloat(-1.0, 0.0, 0.0)
      }
    } else {
      if velocity.Y.N < 0 {
        result.Normal = fixpoint.Vec3Q16FromFloat(0.0, 1.0, 0.0)
      } else {
        result.Normal = fixpoint.Vec3Q16FromFloat(0.0, -1.0, 0.0)
      }
    }
  }

  return result
}
