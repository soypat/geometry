package ms2

import (
	"math/rand"
	"testing"

	math "github.com/chewxy/math32"
)

// TestLineDistanceInfinite2 checks DistanceInfinite2 equals DistanceInfinite squared.
func TestLineDistanceInfinite2(t *testing.T) {
	lines := []Line{
		{{0, 0}, {1, 0}},        // horizontal
		{{0, 0}, {0, 1}},        // vertical
		{{-1, -1}, {2, 2}},      // diagonal through origin
		{{3, -2}, {-5, 7}},      // arbitrary
		{{100, 100}, {103, 99}}, // offset from origin
	}
	points := []Vec{
		{0, 0}, {1, 1}, {-3, 4}, {0.5, 0.5}, {10, -10}, {101, 102},
	}
	for _, ln := range lines {
		for _, p := range points {
			d := ln.DistanceInfinite(p)
			d2 := ln.DistanceInfinite2(p)
			want := d * d
			// Relative tolerance: DistanceInfinite2 is degree-4 in coordinates so
			// it accumulates more float32 rounding than the Hypot-based version.
			tol := 1e-4 * (1 + math.Abs(want))
			if math.Abs(d2-want) > tol {
				t.Errorf("line %v point %v: DistanceInfinite2=%g want DistanceInfinite²=%g", ln, p, d2, want)
			}
		}
	}
}

// TestLineDistanceInfinite2Random fuzzes the relationship d2 == d² over random lines and points.
func TestLineDistanceInfinite2Random(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	randVec := func() Vec {
		return Vec{X: float32(rng.Float64()*20 - 10), Y: float32(rng.Float64()*20 - 10)}
	}
	for i := 0; i < 1000; i++ {
		a, b := randVec(), randVec()
		if Norm2(Sub(b, a)) < 1e-6 {
			continue // skip degenerate (near zero-length) lines
		}
		ln := Line{a, b}
		p := randVec()
		d := ln.DistanceInfinite(p)
		d2 := ln.DistanceInfinite2(p)
		want := d * d
		tol := 1e-3 * (1 + want)
		if math.Abs(d2-want) > tol {
			t.Fatalf("iter %d: line %v point %v: DistanceInfinite2=%g want %g (diff %g)", i, ln, p, d2, want, d2-want)
		}
	}
}

// TestLineDistanceInfinite2OnLine confirms a point exactly on the line has zero squared distance.
func TestLineDistanceInfinite2OnLine(t *testing.T) {
	ln := Line{{1, 2}, {4, 8}}
	for _, ts := range []float32{-0.5, 0, 0.3, 1, 2.0} {
		p := ln.Interpolate(ts)
		if d2 := ln.DistanceInfinite2(p); math.Abs(d2) > 1e-4 {
			t.Errorf("point on line at t=%g should have ~0 squared distance, got %g", ts, d2)
		}
	}
}

// TestLineClosest checks Line.Closest against its documented contract:
// vertexOrSegment is 0 or 1 when closest to vertex l[0] or l[1] respectively,
// and -1 when closest to the segment interior.
func TestLineClosest(t *testing.T) {
	ln := Line{{0, 0}, {4, 0}} // l[0]=(0,0), l[1]=(4,0)
	cases := []struct {
		p    Vec
		want Vec
		flag int8
	}{
		{Vec{-1, -1}, Vec{0, 0}, 0}, // beyond l[0] -> flag 0
		{Vec{5, 1}, Vec{4, 0}, 1},   // beyond l[1] -> flag 1
		{Vec{2, 3}, Vec{2, 0}, -1},  // over the segment -> flag -1
	}
	for _, tc := range cases {
		c, flag := ln.Closest(tc.p)
		if !EqualElem(c, tc.want, 1e-5) {
			t.Errorf("p=%v: closest=%v want %v", tc.p, c, tc.want)
		}
		if flag != tc.flag {
			t.Errorf("p=%v: vertexOrSegment=%d want %d", tc.p, flag, tc.flag)
		}
	}
}

// bruteForceClosest finds the closest point on the triangle boundary to p by
// densely sampling each edge. Independent oracle for Triangle.Closest's point.
func bruteForceClosest(tri Triangle, p Vec) (best Vec, bestD2 float32) {
	const n = 4000
	bestD2 = math.MaxFloat32
	for j := 0; j < 3; j++ {
		edge := Line{tri[j], tri[(j+1)%3]}
		for i := 0; i <= n; i++ {
			q := edge.Interpolate(float32(i) / n)
			if d2 := Norm2(Sub(p, q)); d2 < bestD2 {
				bestD2 = d2
				best = q
			}
		}
	}
	return best, bestD2
}

func TestTriangleClosestInside(t *testing.T) {
	tris := []Triangle{
		{{0, 0}, {4, 0}, {0, 4}},
		{{-2, -1}, {3, -2}, {1, 5}},
	}
	for _, tri := range tris {
		// Centroid and points pulled toward it from each vertex are interior.
		cen := tri.Centroid()
		inside := []Vec{cen}
		for _, v := range tri {
			inside = append(inside, Add(Scale(0.5, v), Scale(0.5, cen)))
		}
		for _, p := range inside {
			c, side, vtx := tri.Closest(p)
			if side != -1 || vtx != -1 {
				t.Errorf("interior point %v of %v: want side=vertex=-1, got side=%d vertex=%d", p, tri, side, vtx)
			}
			if !EqualElem(c, p, 1e-5) {
				t.Errorf("interior point %v: closest should equal p, got %v", p, c)
			}
		}
	}
}

func TestTriangleClosestPoint(t *testing.T) {
	tris := []Triangle{
		{{0, 0}, {4, 0}, {0, 4}},
		{{-2, -1}, {3, -2}, {1, 5}},
		{{10, 10}, {12, 9}, {11, 14}},
	}
	rng := rand.New(rand.NewSource(7))
	for _, tri := range tris {
		cen := tri.Centroid()
		for i := 0; i < 300; i++ {
			// Spread points around the triangle, mostly outside.
			p := Add(cen, Vec{X: float32(rng.Float64()*30 - 15), Y: float32(rng.Float64()*30 - 15)})
			if tri.Contains(p) {
				continue
			}
			c, side, vtx := tri.Closest(p)

			// Exactly one of side/vertex is non-negative for an exterior point.
			if (side >= 0) == (vtx >= 0) {
				t.Errorf("tri %v p=%v: exactly one of side/vertex must be >=0, got side=%d vertex=%d", tri, p, side, vtx)
			}
			// When a side is reported, the closest point lies on that infinite side.
			if side >= 0 {
				edge := Line{tri[side], tri[(side+1)%3]}
				if d2 := edge.DistanceInfinite2(c); d2 > 1e-4 {
					t.Errorf("tri %v p=%v: closest %v not on reported side %d (d²=%g)", tri, p, c, side, d2)
				}
			}
			// When a vertex is reported, the closest point is that vertex.
			if vtx >= 0 && !EqualElem(c, tri[vtx], 1e-4) {
				t.Errorf("tri %v p=%v: closest %v != reported vertex %d %v", tri, p, c, vtx, tri[vtx])
			}

			// The returned closest point must match the brute-force boundary minimum.
			ref, refD2 := bruteForceClosest(tri, p)
			gotD2 := Norm2(Sub(p, c))
			// gotD2 should be no worse than the sampled reference (allow sampling slack).
			tol := 1e-2 * (1 + refD2)
			if gotD2 > refD2+tol {
				t.Errorf("tri %v p=%v: closest %v (d²=%g) worse than brute force %v (d²=%g)", tri, p, c, gotD2, ref, refD2)
			}
		}
	}
}

// TestTriangleClosestKnown checks the closest point against hand-computed values.
func TestTriangleClosestKnown(t *testing.T) {
	tri := Triangle{{0, 0}, {4, 0}, {0, 4}}
	cases := []struct {
		p, want Vec
	}{
		{Vec{2, -1}, Vec{2, 0}},  // below side AB
		{Vec{-1, 2}, Vec{0, 2}},  // left of side CA
		{Vec{-1, -1}, Vec{0, 0}}, // beyond vertex A
		{Vec{5, -1}, Vec{4, 0}},  // beyond vertex B
		{Vec{-1, 5}, Vec{0, 4}},  // beyond vertex C
		{Vec{5, 5}, Vec{2, 2}},   // beyond hypotenuse BC
	}
	for _, tc := range cases {
		c, _, _ := tri.Closest(tc.p)
		if !EqualElem(c, tc.want, 1e-4) {
			t.Errorf("p=%v: closest=%v want %v", tc.p, c, tc.want)
		}
	}
}

// TestTriangleClosestFlags verifies the side/vertex contract: exactly one is
// non-negative for an exterior point, identifying the closest side or vertex.
func TestTriangleClosestFlags(t *testing.T) {
	tri := Triangle{{0, 0}, {4, 0}, {0, 4}}
	cases := []struct {
		p         Vec
		side, vtx int8
	}{
		{Vec{2, -1}, 0, -1},  // closest on side 0 (AB)
		{Vec{5, 5}, 1, -1},   // closest on side 1 (BC, hypotenuse)
		{Vec{-1, 2}, 2, -1},  // closest on side 2 (CA)
		{Vec{-1, -1}, -1, 0}, // closest to vertex 0 (A)
		{Vec{5, -1}, -1, 1},  // closest to vertex 1 (B)
		{Vec{-1, 5}, -1, 2},  // closest to vertex 2 (C)
	}
	for _, tc := range cases {
		_, side, vtx := tri.Closest(tc.p)
		if side != tc.side || vtx != tc.vtx {
			t.Errorf("p=%v: got side=%d vertex=%d want side=%d vertex=%d", tc.p, side, vtx, tc.side, tc.vtx)
		}
	}
}
