package spacesim

import(
  "github.com/ezmicken/fixpoint"
)

type Collider struct {
  broadSize       fixpoint.Q16
  narrowSize      fixpoint.Q16
  halfBroadSize   fixpoint.Q16
  halfNarrowSize  fixpoint.Q16

  Broad           Rect
  Narrow          Rect
}

type collision struct {
  Time    fixpoint.Q16
  Normal  fixpoint.Vec3Q16
  Area    fixpoint.Q16
}

func NewCollider(broadSize, narrowSize int32) *Collider {
  var c Collider
  c.broadSize = fixpoint.Q16FromInt32(broadSize)
  c.narrowSize = fixpoint.Q16FromInt32(narrowSize)
  c.halfBroadSize = fixpoint.Q16FromInt32(broadSize/2)
  c.halfNarrowSize = fixpoint.Q16FromInt32(narrowSize/2)

  c.Broad = NewRect(c.halfBroadSize, c.halfBroadSize, c.broadSize, c.broadSize)
  c.Narrow = NewRect(c.halfNarrowSize, c.halfNarrowSize, c.narrowSize, c.narrowSize)

  return &c
}

func (c *Collider) Update(position, velocity fixpoint.Vec3Q16) {
  c.Broad.Min = Point{ position.X.Sub(c.halfBroadSize), position.Y.Sub(c.halfBroadSize)}
  c.Broad.Max = Point{ position.X.Add(c.halfBroadSize), position.Y.Add(c.halfBroadSize)}
  c.Narrow.Min = Point{ position.X.Sub(c.halfNarrowSize), position.Y.Sub(c.halfNarrowSize)}
  c.Narrow.Max = Point{ position.X.Add(c.halfNarrowSize), position.Y.Add(c.halfNarrowSize)}
}

func (c *Collider) Check(ht HistoricalTransform, potentialCollisions []Rect) HistoricalTransform {
  var closest collision
  closest.Time = fixpoint.OneQ16
  closest.Normal = fixpoint.ZeroVec3Q16
  closest.Area = fixpoint.ZeroQ16

  for i := 0; i < len(potentialCollisions); i++ {
    if RectOverlap(c.Broad, potentialCollisions[i]).N > fixpoint.ZeroQ16.N {
      col := c.sweep(ht.Velocity, potentialCollisions[i])
      if col.Area.N > closest.Area.N {
        closest = col
      }
    }
  }

  pos := ht.Position
  vel := ht.Velocity

  remainingTime := fixpoint.OneQ16.Sub(closest.Time)
  threshold := fixpoint.Q16FromFloat(0.001)
  if remainingTime.N > fixpoint.ZeroQ16.N {
    if fixpoint.Abs(closest.Normal.X).N > threshold.N {
      if fixpoint.Abs(vel.X).N < fixpoint.OneQ16.N {
        pos.X = pos.X.Add(vel.X.Mul(closest.Time))
        vel.X = fixpoint.ZeroQ16
      } else {
        pos.X = pos.X.Add(vel.X.Mul(closest.Time))
        vel.X = vel.X.Mul(fixpoint.HalfQ16).Neg()
        pos.X = pos.X.Add(vel.Y.Mul(remainingTime))
      }
    }

    if fixpoint.Abs(closest.Normal.Y).N > threshold.N {
      if fixpoint.Abs(vel.Y).N < fixpoint.OneQ16.N {
        pos.Y = pos.Y.Add(vel.Y.Mul(closest.Time))
        vel.Y = fixpoint.ZeroQ16
      } else {
        pos.Y = pos.Y.Add(vel.Y.Mul(closest.Time))
        vel.Y = vel.Y.Mul(fixpoint.HalfQ16).Neg()
        pos.Y = pos.Y.Add(vel.Y.Mul(remainingTime))
      }
    }

    ht.VelocityDelta = vel.Sub(ht.Velocity.Sub(ht.VelocityDelta))
    ht.Position = pos
    ht.Velocity = vel

    c.Update(ht.Position, ht.Velocity)
  }

  return ht
}

func (c *Collider) sweep(velocity fixpoint.Vec3Q16, block Rect) collision {
  var dxEntry fixpoint.Q16
  var dxExit fixpoint.Q16
  var dyEntry fixpoint.Q16
  var dyExit fixpoint.Q16
  var result collision

  if velocity.X.N > fixpoint.ZeroQ16.N {
    dxEntry = block.Min.X.Sub(c.Narrow.Max.X)
    dxExit = block.Max.X.Sub(c.Narrow.Min.X)
  } else {
    dxEntry = block.Max.X.Sub(c.Narrow.Min.X)
    dxExit = block.Min.X.Sub(c.Narrow.Max.X)
  }

  if velocity.Y.N > fixpoint.ZeroQ16.N {
    dyEntry = block.Min.Y.Sub(c.Narrow.Max.Y)
    dyExit = block.Max.Y.Sub(c.Narrow.Min.Y)
  } else {
    dyEntry = block.Max.Y.Sub(c.Narrow.Min.Y)
    dyExit = block.Min.Y.Sub(c.Narrow.Max.Y)
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

  if exiting || negativeEntry || futureEntry {
    result.Time = fixpoint.OneQ16
    result.Normal = fixpoint.ZeroVec3Q16
  } else {
    result.Time = entryTime
    movedBody := NewRect(c.Narrow.Min.X.Add(velocity.X), c.Narrow.Min.Y.Add(velocity.Y), c.Narrow.W, c.Narrow.H)
    result.Area = RectOverlap(movedBody, block)
    if txEntry.N > tyEntry.N {
      if dxEntry.N < 0 {
        result.Normal = fixpoint.Vec3Q16FromFloat(1.0, 0.0, 0.0)
      } else {
        result.Normal = fixpoint.Vec3Q16FromFloat(-1.0, 0.0, 0.0)
      }
    } else {
      if dyEntry.N < 0 {
        result.Normal = fixpoint.Vec3Q16FromFloat(0.0, 1.0, 0.0)
      } else {
        result.Normal = fixpoint.Vec3Q16FromFloat(0.0, -1.0, 0.0)
      }
    }
  }

  return result
}
