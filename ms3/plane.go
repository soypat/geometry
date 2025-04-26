package ms3

import math "github.com/chewxy/math32"

// Plane represents a plane in 3D space. To safely construct one use [Triangle.Plane]
type Plane struct {
	// p is a point on the plane
	p Vec
	// n is the unit vector normal to the plane.
	n Vec
}

func newPlane(p, n Vec) Plane {
	return Plane{p: p, n: Unit(n)}
}

func (p Plane) distanceTo(q Vec) float32 {
	v := Sub(q, p.p)
	return math.Abs(Dot(v, p.n))
}
