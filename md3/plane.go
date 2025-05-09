// DO NOT EDIT.
// This file was generated automatically
// from gen.go. Please do not edit this file.

package md3

import math "math"

// Plane represents an infinite plane in 3D space. To safely construct one use [Triangle.Plane]
type Plane struct {
	// p is a point on the plane
	p Vec
	// n is the unit vector normal to the plane.
	n Vec
}

func newPlane(p, n Vec) Plane {
	return Plane{p: p, n: Unit(n)}
}

// Distance returns the minimum euclidean distance from the infinite plane to the argument point.
func (p Plane) Distance(point Vec) float64 {
	v := Sub(point, p.p)
	return math.Abs(Dot(v, p.n))
}
