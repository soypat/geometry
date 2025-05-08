package i2

type Vec struct {
	X int
	Y int
}

func (a Vec) Add(b Vec) Vec { return Vec{X: a.X + b.X, Y: a.Y + b.Y} }
func (a Vec) Sub(v Vec) Vec { return Vec{X: a.X - v.X, Y: a.Y - v.Y} }

func (a Vec) AddScalar(v int) Vec { return Vec{X: a.X + v, Y: a.Y + v} }
func (a Vec) MulScalar(v int) Vec { return Vec{X: a.X * v, Y: a.Y * v} }
func (a Vec) DivScalar(v int) Vec { return Vec{X: a.X / v, Y: a.Y / v} }

func (a Vec) ShiftRight(lo int) Vec { return Vec{X: a.X >> lo, Y: a.Y >> lo} }
func (a Vec) ShiftLeft(hi int) Vec  { return Vec{X: a.X << hi, Y: a.Y << hi} }

func (a Vec) AndScalar(b int) Vec    { return Vec{X: a.X & b, Y: a.Y & b} }
func (a Vec) OrScalar(b int) Vec     { return Vec{X: a.X | b, Y: a.Y | b} }
func (a Vec) XorScalar(b int) Vec    { return Vec{X: a.X ^ b, Y: a.Y ^ b} }
func (a Vec) AndnotScalar(b int) Vec { return Vec{X: a.X &^ b, Y: a.Y &^ b} }
