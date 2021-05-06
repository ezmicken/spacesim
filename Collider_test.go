package spacesim

import(
  "testing"
  "github.com/ezmicken/fixpoint"
)

// -32/2112
func TestDiagonalCheck(t *testing.T) {
  collider := NewCollider(48)

  ht := HistoricalTransform{1958, 144, 0, fixpoint.Vec3Q16{fixpoint.Q16{1881528}, fixpoint.Q16{142981676}, fixpoint.Q16{0}}, fixpoint.Vec3Q16{fixpoint.Q16{-767140}, fixpoint.Q16{-1062780}, fixpoint.Q16{0}}, fixpoint.Vec3Q16{fixpoint.Q16{620}, fixpoint.Q16{1000}, fixpoint.Q16{0}}}
  collider.Update(ht.Position, ht.Velocity)
  potentialCollisions := []Rect{
    {Point{fixpoint.Q16{132120576}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{134217728}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{134217728}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{136314880}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{136314880}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{138412032}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{138412032}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{140509184}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{140509184}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{142606336}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{127926272}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{130023424}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{130023424}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{132120576}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{132120576}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{134217728}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{134217728}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{136314880}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{136314880}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{138412032}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{138412032}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{140509184}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{140509184}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{142606336}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{127926272}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{130023424}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{130023424}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{132120576}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{132120576}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{134217728}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{134217728}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{136314880}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{136314880}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{138412032}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{138412032}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{140509184}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{140509184}, fixpoint.Q16{163577856}}, Point{fixpoint.Q16{142606336}, fixpoint.Q16{165675008}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{29360128}, fixpoint.Q16{182452224}}, Point{fixpoint.Q16{31457280}, fixpoint.Q16{184549376}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{29360128}, fixpoint.Q16{184549376}}, Point{fixpoint.Q16{31457280}, fixpoint.Q16{186646528}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{146800640}}, Point{fixpoint.Q16{0}, fixpoint.Q16{148897792}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{148897792}}, Point{fixpoint.Q16{0}, fixpoint.Q16{150994944}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{150994944}}, Point{fixpoint.Q16{0}, fixpoint.Q16{153092096}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{153092096}}, Point{fixpoint.Q16{0}, fixpoint.Q16{155189248}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{155189248}}, Point{fixpoint.Q16{0}, fixpoint.Q16{157286400}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{157286400}}, Point{fixpoint.Q16{0}, fixpoint.Q16{159383552}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{0}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{144703488}}, Point{fixpoint.Q16{0}, fixpoint.Q16{146800640}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{144703488}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{146800640}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{146800640}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{148897792}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{148897792}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{150994944}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{150994944}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{153092096}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{153092096}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{155189248}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{155189248}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{157286400}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{157286400}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{159383552}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{142606336}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{144703488}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{142606336}}, Point{fixpoint.Q16{0}, fixpoint.Q16{144703488}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{142606336}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{144703488}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{144703488}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{146800640}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{146800640}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{148897792}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{148897792}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{150994944}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{150994944}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{153092096}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{153092096}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{155189248}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{155189248}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{157286400}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{140509184}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{142606336}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{140509184}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{142606336}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{140509184}}, Point{fixpoint.Q16{0}, fixpoint.Q16{142606336}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{138412032}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{140509184}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{138412032}}, Point{fixpoint.Q16{0}, fixpoint.Q16{140509184}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{136314880}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{138412032}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{136314880}}, Point{fixpoint.Q16{0}, fixpoint.Q16{138412032}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{134217728}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{136314880}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{134217728}}, Point{fixpoint.Q16{0}, fixpoint.Q16{136314880}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{132120576}}, Point{fixpoint.Q16{0}, fixpoint.Q16{134217728}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{130023424}}, Point{fixpoint.Q16{0}, fixpoint.Q16{132120576}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{127926272}}, Point{fixpoint.Q16{0}, fixpoint.Q16{130023424}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{125829120}}, Point{fixpoint.Q16{0}, fixpoint.Q16{127926272}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{2097152}, fixpoint.Q16{117440512}}, Point{fixpoint.Q16{4194304}, fixpoint.Q16{119537664}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{4194304}, fixpoint.Q16{117440512}}, Point{fixpoint.Q16{6291456}, fixpoint.Q16{119537664}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{2097152}, fixpoint.Q16{115343360}}, Point{fixpoint.Q16{4194304}, fixpoint.Q16{117440512}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{4194304}, fixpoint.Q16{115343360}}, Point{fixpoint.Q16{6291456}, fixpoint.Q16{117440512}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{6291456}, fixpoint.Q16{115343360}}, Point{fixpoint.Q16{8388608}, fixpoint.Q16{117440512}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{2097152}, fixpoint.Q16{113246208}}, Point{fixpoint.Q16{4194304}, fixpoint.Q16{115343360}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{4194304}, fixpoint.Q16{113246208}}, Point{fixpoint.Q16{6291456}, fixpoint.Q16{115343360}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{6291456}, fixpoint.Q16{113246208}}, Point{fixpoint.Q16{8388608}, fixpoint.Q16{115343360}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{4194304}, fixpoint.Q16{111149056}}, Point{fixpoint.Q16{6291456}, fixpoint.Q16{113246208}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{6291456}, fixpoint.Q16{111149056}}, Point{fixpoint.Q16{8388608}, fixpoint.Q16{113246208}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{4194304}, fixpoint.Q16{109051904}}, Point{fixpoint.Q16{6291456}, fixpoint.Q16{111149056}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{6291456}, fixpoint.Q16{109051904}}, Point{fixpoint.Q16{8388608}, fixpoint.Q16{111149056}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{4194304}, fixpoint.Q16{106954752}}, Point{fixpoint.Q16{6291456}, fixpoint.Q16{109051904}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{31457280}, fixpoint.Q16{180355072}}, Point{fixpoint.Q16{33554432}, fixpoint.Q16{182452224}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{31457280}, fixpoint.Q16{182452224}}, Point{fixpoint.Q16{33554432}, fixpoint.Q16{184549376}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{31457280}, fixpoint.Q16{184549376}}, Point{fixpoint.Q16{33554432}, fixpoint.Q16{186646528}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{138412032}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{140509184}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{136314880}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{138412032}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{132120576}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{134217728}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{130023424}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{132120576}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{127926272}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{130023424}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{123731968}}, Point{fixpoint.Q16{0}, fixpoint.Q16{125829120}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{121634816}}, Point{fixpoint.Q16{0}, fixpoint.Q16{123731968}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{119537664}}, Point{fixpoint.Q16{0}, fixpoint.Q16{121634816}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-2097152}, fixpoint.Q16{117440512}}, Point{fixpoint.Q16{0}, fixpoint.Q16{119537664}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{0}, fixpoint.Q16{117440512}}, Point{fixpoint.Q16{2097152}, fixpoint.Q16{119537664}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{0}, fixpoint.Q16{115343360}}, Point{fixpoint.Q16{2097152}, fixpoint.Q16{117440512}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{0}, fixpoint.Q16{113246208}}, Point{fixpoint.Q16{2097152}, fixpoint.Q16{115343360}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{16777216}, fixpoint.Q16{125829120}}, Point{fixpoint.Q16{18874368}, fixpoint.Q16{127926272}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{18874368}, fixpoint.Q16{127926272}}, Point{fixpoint.Q16{20971520}, fixpoint.Q16{130023424}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{18874368}, fixpoint.Q16{130023424}}, Point{fixpoint.Q16{20971520}, fixpoint.Q16{132120576}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{20971520}, fixpoint.Q16{132120576}}, Point{fixpoint.Q16{23068672}, fixpoint.Q16{134217728}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{33554432}, fixpoint.Q16{180355072}}, Point{fixpoint.Q16{35651584}, fixpoint.Q16{182452224}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{33554432}, fixpoint.Q16{182452224}}, Point{fixpoint.Q16{35651584}, fixpoint.Q16{184549376}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{33554432}, fixpoint.Q16{184549376}}, Point{fixpoint.Q16{35651584}, fixpoint.Q16{186646528}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{29360128}, fixpoint.Q16{186646528}}, Point{fixpoint.Q16{31457280}, fixpoint.Q16{188743680}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{31457280}, fixpoint.Q16{186646528}}, Point{fixpoint.Q16{33554432}, fixpoint.Q16{188743680}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{33554432}, fixpoint.Q16{186646528}}, Point{fixpoint.Q16{35651584}, fixpoint.Q16{188743680}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{35651584}, fixpoint.Q16{180355072}}, Point{fixpoint.Q16{37748736}, fixpoint.Q16{182452224}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{35651584}, fixpoint.Q16{182452224}}, Point{fixpoint.Q16{37748736}, fixpoint.Q16{184549376}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{35651584}, fixpoint.Q16{184549376}}, Point{fixpoint.Q16{37748736}, fixpoint.Q16{186646528}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{35651584}, fixpoint.Q16{186646528}}, Point{fixpoint.Q16{37748736}, fixpoint.Q16{188743680}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{37748736}, fixpoint.Q16{180355072}}, Point{fixpoint.Q16{39845888}, fixpoint.Q16{182452224}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{37748736}, fixpoint.Q16{182452224}}, Point{fixpoint.Q16{39845888}, fixpoint.Q16{184549376}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{37748736}, fixpoint.Q16{184549376}}, Point{fixpoint.Q16{39845888}, fixpoint.Q16{186646528}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{37748736}, fixpoint.Q16{186646528}}, Point{fixpoint.Q16{39845888}, fixpoint.Q16{188743680}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{132120576}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{134217728}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{134217728}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{136314880}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-6291456}, fixpoint.Q16{130023424}}, Point{fixpoint.Q16{-4194304}, fixpoint.Q16{132120576}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{-4194304}, fixpoint.Q16{125829120}}, Point{fixpoint.Q16{-2097152}, fixpoint.Q16{127926272}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{18874368}, fixpoint.Q16{123731968}}, Point{fixpoint.Q16{20971520}, fixpoint.Q16{125829120}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{18874368}, fixpoint.Q16{125829120}}, Point{fixpoint.Q16{20971520}, fixpoint.Q16{127926272}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{20971520}, fixpoint.Q16{125829120}}, Point{fixpoint.Q16{23068672}, fixpoint.Q16{127926272}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{20971520}, fixpoint.Q16{127926272}}, Point{fixpoint.Q16{23068672}, fixpoint.Q16{130023424}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{20971520}, fixpoint.Q16{130023424}}, Point{fixpoint.Q16{23068672}, fixpoint.Q16{132120576}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{23068672}, fixpoint.Q16{130023424}}, Point{fixpoint.Q16{25165824}, fixpoint.Q16{132120576}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{23068672}, fixpoint.Q16{132120576}}, Point{fixpoint.Q16{25165824}, fixpoint.Q16{134217728}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{23068672}, fixpoint.Q16{134217728}}, Point{fixpoint.Q16{25165824}, fixpoint.Q16{136314880}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{25165824}, fixpoint.Q16{132120576}}, Point{fixpoint.Q16{27262976}, fixpoint.Q16{134217728}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
    {Point{fixpoint.Q16{25165824}, fixpoint.Q16{134217728}}, Point{fixpoint.Q16{27262976}, fixpoint.Q16{136314880}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
  }

  result := collider.Check(ht.Position, ht.Velocity, potentialCollisions)

  // expect deflect in X axis
  if result.Normal.X.N < fixpoint.ZeroQ16.N {
  //if result.Velocity.X != ht.Velocity.X.Mul(fixpoint.HalfQ16).Neg() {
    t.Logf("SW Diagonal collision not resolved as expected. %v %v", result.Normal.X.Float(), result.Normal.Y.Float())
    t.Fail()
  }
}

// func TestRightCheck(t *testing.T) {
//   collider := NewCollider(48)
//   ht := HistoricalTransform{339, 0, 0, fixpoint.Vec3Q16{fixpoint.Q16{135266304}, fixpoint.Q16{157159680}, fixpoint.Q16{0}}, fixpoint.Vec3Q16{fixpoint.Q16{0}, fixpoint.Q16{684168}, fixpoint.Q16{0}}, fixpoint.Vec3Q16{fixpoint.Q16{0}, fixpoint.Q16{0}, fixpoint.Q16{0}}}
//   collider.Update(ht.Position, ht.Velocity)
//   potentialCollisions := []Rect{
//     {Point{fixpoint.Q16{132120576}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{134217728}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{134217728}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{136314880}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{136314880}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{138412032}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{138412032}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{140509184}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{140509184}, fixpoint.Q16{159383552}}, Point{fixpoint.Q16{142606336}, fixpoint.Q16{161480704}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{127926272}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{130023424}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{130023424}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{132120576}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{132120576}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{134217728}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{134217728}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{136314880}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{136314880}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{138412032}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{138412032}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{140509184}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//     {Point{fixpoint.Q16{140509184}, fixpoint.Q16{161480704}}, Point{fixpoint.Q16{142606336}, fixpoint.Q16{163577856}}, fixpoint.Q16{2097152}, fixpoint.Q16{2097152}},
//   }

//   result := collider.Check(ht, potentialCollisions)

//   // expect deflect in X axis
//   if result.Velocity.X != ht.Velocity.X.Mul(fixpoint.HalfQ16).Neg() {
//     t.Logf("E straight collision was not resolved as expected.")
//     t.Fail()
//   }
// }

// collider is touching one block
func TestSlideCheck(t *testing.T) {
  inputPosition := fixpoint.Vec3Q16{fourtyEight, fourtyEight, zero}
  inputVelocity := fixpoint.Vec3Q16{zero, ten, zero}
  expectedNormal := fixpoint.Vec3Q16{zero, fixpoint.OneQ16.Neg(), zero}
  collider := NewCollider(32)

  collider.Update(inputPosition, inputVelocity)
  block := NewRect(thirtyTwo, sixtyFour, thirtyTwo, thirtyTwo)

  ht := HistoricalTransform{339, 0, 0, inputPosition, inputVelocity, fixpoint.ZeroVec3Q16}

  result := collider.Check(ht.Position, ht.Velocity, []Rect{block})

  if result.Normal != expectedNormal {
    t.Logf("Overlap test did not get expected result: %v/%v", result.Normal.X.Float(), result.Normal.Y.Float())
    t.Fail()
  }
}

func TestFixpointStuff(t *testing.T) {
  vel := fixpoint.Vec3Q16{zero, ten, zero}
  t.Logf("%v %v", vel.Y.Float(), vel.Y.Mul(fixpoint.Q16{3}).Float())
}

func TestExposedNormalCheck(t *testing.T) {
  inputPosition := fixpoint.Vec3Q16{fourtySeven, eighty, zero}
  inputVelocity := fixpoint.Vec3Q16{five, fixpoint.Q16FromFloat(7).Neg(), zero}
  expectedNormal := fixpoint.Vec3Q16{zero, fixpoint.OneQ16, zero}
  collider := NewCollider(32)

  collider.Update(inputPosition, inputVelocity)
  block1 := NewRect(thirtyTwo, thirtyTwo, thirtyTwo, thirtyTwo)
  block2 := NewRect(sixtyFour, thirtyTwo, thirtyTwo, thirtyTwo)
  block3 := NewRect(ninetySix, thirtyTwo, thirtyTwo, thirtyTwo)

  ht := HistoricalTransform{339, 0, 0, inputPosition, inputVelocity, fixpoint.ZeroVec3Q16}

  result := collider.Check(ht.Position, ht.Velocity, []Rect{block1, block2, block3})

  if result.Normal != expectedNormal {
    t.Logf("exposed normal test did not get expected result: %v/%v", result.Normal.X.Float(), result.Normal.Y.Float())
    t.Fail()
  }
}

func TestFastObject(t *testing.T) {
  inputPosition := fixpoint.Vec3Q16{fourtyEight, thirtyTwo, zero}
  inputVelocity := fixpoint.Vec3Q16{zero, oneSixty, zero}
  expectedNormal := fixpoint.Vec3Q16{zero, one.Neg(), zero}
  collider := NewCollider(32)

  collider.Update(inputPosition, inputVelocity)
  block := NewRect(thirtyTwo, sixtyFour, thirtyTwo, thirtyTwo)

  ht := HistoricalTransform{339, 0, 0, inputPosition, inputVelocity, fixpoint.ZeroVec3Q16}

  result := collider.Check(ht.Position, ht.Velocity, []Rect{block})

  if result.Normal != expectedNormal {
    t.Logf("Fast object test did not get expected result: %v/%v", result.Normal.X.Float(), result.Normal.Y.Float())
    t.Fail()
  }
}

func TestMissCheck(t *testing.T) {
  inputPosition := fixpoint.Vec3Q16{fourtyEight, fourtyEight, zero}
  inputVelocity := fixpoint.Vec3Q16{zero, ten, zero}
  expectedNormal := fixpoint.ZeroVec3Q16
  collider := NewCollider(32)

  collider.Update(inputPosition, inputVelocity)
  block := NewRect(oneSixty, sixtyFour, thirtyTwo, thirtyTwo)

  ht := HistoricalTransform{339, 0, 0, inputPosition, inputVelocity, fixpoint.ZeroVec3Q16}

  result := collider.Check(ht.Position, ht.Velocity, []Rect{block})

  if result.Normal != expectedNormal {
    t.Logf("Overlap test did not get expected result: %v/%v", result.Normal.X.Float(), result.Normal.Y.Float())
    t.Fail()
  }
}
