package spacesim

import(
  "github.com/ezmicken/fixpoint"
)

var timestep fixpoint.Q16 = fixpoint.Q16FromInt32(33)
var scale fixpoint.Q16 = fixpoint.Q16FromInt32(32)

// some number constants
var zero        fixpoint.Q16  = fixpoint.ZeroQ16
var one         fixpoint.Q16  = fixpoint.OneQ16
var two         fixpoint.Q16  = fixpoint.TwoQ16
var five        fixpoint.Q16  = fixpoint.Q16FromInt32(5)
var ten         fixpoint.Q16  = fixpoint.Q16FromInt32(10)
var twenty      fixpoint.Q16  = fixpoint.Q16FromInt32(20)
var twentyNine  fixpoint.Q16  = fixpoint.Q16FromInt32(29)
var thirty      fixpoint.Q16  = fixpoint.Q16FromInt32(30)
var thirtyTwo   fixpoint.Q16  = fixpoint.Q16FromInt32(32)
var thirtyFour  fixpoint.Q16  = fixpoint.Q16FromInt32(34)
var thirtyFive  fixpoint.Q16  = fixpoint.Q16FromInt32(35)
var thirtyNine  fixpoint.Q16  = fixpoint.Q16FromInt32(39)
var fourtyThree fixpoint.Q16  = fixpoint.Q16FromInt32(43)
var fourtyFive  fixpoint.Q16  = fixpoint.Q16FromInt32(45)
var fourtySix   fixpoint.Q16  = fixpoint.Q16FromInt32(46)
var fourtySeven fixpoint.Q16  = fixpoint.Q16FromInt32(47)
var fourtyEight fixpoint.Q16  = fixpoint.Q16FromInt32(48)
var fourtyNine  fixpoint.Q16  = fixpoint.Q16FromInt32(49)
var fifty       fixpoint.Q16  = fixpoint.Q16FromInt32(50)
var fiftyOne    fixpoint.Q16  = fixpoint.Q16FromInt32(51)
var fiftyTwo    fixpoint.Q16  = fixpoint.Q16FromInt32(52)
var fiftyFive   fixpoint.Q16  = fixpoint.Q16FromInt32(55)
var fiftySeven  fixpoint.Q16  = fixpoint.Q16FromInt32(57)
var fiftyEight  fixpoint.Q16  = fixpoint.Q16FromInt32(58)
var sixtyTwo    fixpoint.Q16  = fixpoint.Q16FromInt32(62)
var sixtyFour   fixpoint.Q16  = fixpoint.Q16FromInt32(64)
var sixtySeven  fixpoint.Q16  = fixpoint.Q16FromInt32(67)
var seventyNine fixpoint.Q16  = fixpoint.Q16FromInt32(79)
var eighty      fixpoint.Q16  = fixpoint.Q16FromInt32(80)
var eightyOne   fixpoint.Q16  = fixpoint.Q16FromInt32(81)
var eightyThree fixpoint.Q16  = fixpoint.Q16FromInt32(83)
var ninetySix   fixpoint.Q16  = fixpoint.Q16FromInt32(96)
var oneSixty    fixpoint.Q16  = fixpoint.Q16FromInt32(160)

var randomHT HistoricalTransform = HistoricalTransform {
  Seq: 500,
  Angle: 0,
  AngleDelta: 0,
  Position: fixpoint.ZeroVec3Q16,
  Velocity: fixpoint.ZeroVec3Q16,
  VelocityDelta: fixpoint.ZeroVec3Q16,
}

var testControlledBodyInfo BodyInfo = BodyInfo {
  Id: 1,
  Size: 48,
  Proximity: 0,
  Lifetime: -1,
  BounceCoefficient: 0.5,
  VelocityX: 0.0,
  VelocityY: 0.0,
}

var testBodyInfo BodyInfo = BodyInfo {
  Id: 1,
  Size: 32,
  Proximity: 0,
  Lifetime: 8000,
  BounceCoefficient: 1,
  VelocityX: 20.0,
  VelocityY: 20.0,
}
