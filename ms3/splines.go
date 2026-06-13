package ms3

type Spline3 struct {
	m Mat4
}

// NewSpline3 returns a [Spline3] ready for use.
// See [Freya Holmér's video] on splines for more information on how a matrix represents a uniform cubic spline.
//
// [Freya Holmér's video]: https://youtu.be/jvPPXbo87ds?si=Sn08aUjSKSXeRZ6D&t=419
func NewSpline3(matrix4x4 []float32) Spline3 {
	if len(matrix4x4) < 16 {
		panic("input matrix too short (need to be 4x4, row major)")
	}
	return Spline3{m: NewMat4(matrix4x4)}
}

// Mat4Array returns a row-major ordered copy of the values of the cubic spline 4x4 matrix.
func (s Spline3) Mat4Array() [16]float32 {
	return s.m.Array()
}

// Evaluate evaluates the cubic spline over 4 points with a value of t. t is usually between 0 and 1 to interpolate the spline.
func (s Spline3) Evaluate(t float32, v0, v1, v2, v3 Vec) (res Vec) {
	// Pack each spatial axis of the 4 control points into a Quat used as a plain
	// 4-vector (I=v0, J=v1, K=v2, W=v3) and apply the spline matrix per axis.
	x := vec4{x: v0.X, y: v1.X, z: v2.X, w: v3.X}
	y := vec4{x: v0.Y, y: v1.Y, z: v2.Y, w: v3.Y}
	z := vec4{x: v0.Z, y: v1.Z, z: v2.Z, w: v3.Z}

	x = matvecmul4(s.m, x)
	y = matvecmul4(s.m, y)
	z = matvecmul4(s.m, z)
	v0 = Vec{X: x.x, Y: y.x, Z: z.x}
	v1 = Vec{X: x.y, Y: y.y, Z: z.y}
	v2 = Vec{X: x.z, Y: y.z, Z: z.z}
	v3 = Vec{X: x.w, Y: y.w, Z: z.w}
	res = Add(v0, Scale(t, v1))
	res = Add(res, Scale(t*t, v2))
	res = Add(res, Scale(t*t*t, v3))
	return res
}

// BasisFuncs returns the basis functions of the cubic spline corresponding to each of 4 control points.
func (s Spline3) BasisFuncs() (bs [4]func(float32) float32) {
	arr := s.m.Transpose().Array()
	for i := range bs {
		off := i * 4
		bs[i] = func(t float32) (b float32) {
			return arr[off+0] + t*arr[off+1] + t*t*arr[off+2] + t*t*t*arr[off+3]
		}
	}
	return bs
}

// BasisFuncs returns the differentiaed basis functions of the cubic spline.
func (s Spline3) BasisFuncsDiff() (bs [4]func(float32) float32) {
	arr := s.m.Transpose().Array()
	for i := range bs {
		off := i * 4
		bs[i] = func(t float32) (b float32) {
			return arr[off+1] + 2*t*arr[off+2] + 3*t*t*arr[off+3]
		}
	}
	return bs
}

// BasisFuncsDiff2 returns the twice-differentiaed basis functions of the cubic spline.
func (s Spline3) BasisFuncsDiff2() (bs [4]func(float32) float32) {
	arr := s.m.Transpose().Array()
	for i := range bs {
		off := i * 4
		bs[i] = func(t float32) (b float32) {
			return 2*arr[off+2] + 6*t*arr[off+3]
		}
	}
	return bs
}

// BasisFuncsDiff3 returns the thrice-differentiaed basis functions of the cubic spline.
func (s Spline3) BasisFuncsDiff3() (bs [4]func(float32) float32) {
	arr := s.m.Transpose().Array()
	for i := range bs {
		off := i * 4
		bs[i] = func(t float32) (b float32) {
			return 6 * arr[off+3]
		}
	}
	return bs
}

// matrix form of bezier curves:
//
//	                        [ a b c d ]   [ P0 ]
//	B(t) = [1  t  t²  t³] * | e f g h | * | P1 |
//	                        | i j k l |   | P2 |
//	                        [ m n o p ]   [ P3 ]
var (
	_beziermat = NewMat4([]float32{
		1, 0, 0, 0,
		-3, 3, 0, 0,
		3, -6, 3, 0,
		-1, 3, -3, 1,
	})
	_hermiteMat = NewMat4([]float32{
		1, 0, 0, 0,
		0, 1, 0, 0,
		-3, -2, 3, -1,
		2, 1, -2, 1,
	})
	_basisMat = ScaleMat4(1./6, NewMat4([]float32{
		1, 4, 1, 0,
		-3, 0, 3, 0,
		3, -6, 3, 0,
		-1, 3, -3, 1,
	}))
	_cardinalMat = func(s float32) Mat4 {
		return NewMat4([]float32{
			0, 1, 0, 0,
			-s, 0, s, 0,
			2 * s, s - 3, 3 - 2*s, -s,
			-s, 2 - s, s - 2, s,
		})
	}
	_catmullromMat      = _cardinalMat(0.5)
	_quadraticBezierMat = NewMat4([]float32{
		1, 0, 0, 0,
		-2, 2, 0, 0,
		1, -2, 1, 0,
		0, 0, 0, 0,
	})
)

// SplineBezierCubic returns a Bézier cubic spline interpreter. Result splines have the following characteristics:
//   - C¹/C⁰ continuous.
//   - Interpolates some points.
//   - Manual tangents, second and third vectors are control points.
//   - Uses in shapes and vector graphics.
//
// Iterate every 3 points. Point0, ControlPoint0, ControlPoint1, Point1.
func SplineBezierCubic() Spline3 { return Spline3{m: _beziermat} }

// SplineHermite returns a Hermite cubic spline interpreter. Result splines have the following characteristics:
//   - C¹/C⁰ continuous.
//   - Interpolates all points.
//   - Explicit tangents. Second and fourth vector arguments specify velocities.
//   - Uses in animation, physics simulations and interpolation.
//
// Iterate every 2 points, Point0, Velocity0, Point1, Velocity1.
func SplineHermite() Spline3 { return Spline3{m: _hermiteMat} }

// SplineCatmullRom returns a Catmull-Rom cubic spline interpreter, a special case of Cardinal spline when scale=0.5. Result splines have the following characteristics:
//   - C¹ continuous.
//   - Interpolates all points.
//   - Automatic tangents.
//   - Used for animation and path smoothing.
func SplineCatmullRom() Spline3 { return Spline3{m: _catmullromMat} }

// SplineCardinal returns a cardinal cubic spline interpreter.
func SplineCardinal(scale float32) Spline3 { return Spline3{m: _cardinalMat(scale)} }

// SplineBasis returns a B-Spline interpreter. Result splines have the following characteristics:
//   - C² continuous.
//   - No point interpolation.
//   - Automatic tangents.
//   - Ideal for curvature-sensitive shapes and animations such as camera paths. Used in industrial design.
func SplineBasis() Spline3 { return Spline3{m: _basisMat} }

// SplineBezierQuadratic returns a quadratic spline interpreter (fourth point is inneffective).
//   - C¹ continuous.
//   - Interpolates all points.
//   - Manual tangents.
//   - Used in fonts. Cubic beziers are superior.
//
// Iterate every 2 points. Point0, ControlPoint, Point1. Keep in mind this is an innefficient implementation of a quadratic bezier. Is here for convenience.
func SplineBezierQuadratic() Spline3 { return Spline3{m: _quadraticBezierMat} }

// Spline3Sampler implements algorithms for sampling points of a cubic spline [Spline3].
type Spline3Sampler struct {
	Spline         Spline3
	v0, v1, v2, v3 Vec
	// Tolerance sets the maximum permissible error for sampling the cubic spline.
	// That is to say the resulting sampled set of line segments will approximate the curve to within Tolerance.
	Tolerance float32
}

// SetSplinePoints sets the 4 [Vec]s which define a cubic spline. They are passed to the Spline on Evaluate calls.
func (s *Spline3Sampler) SetSplinePoints(v0, v1, v2, v3 Vec) {
	s.v0, s.v1, s.v2, s.v3 = v0, v1, v2, v3
}

// Evaluate evaluates a point on the spline with points set by [Spline3Sampler.SetSplinePoints].
// It calls [Spline3.Evaluate] with t and the set points.
func (s *Spline3Sampler) Evaluate(t float32) Vec {
	return s.Spline.Evaluate(t, s.v0, s.v1, s.v2, s.v3)
}

// SampleBisect samples the cubic spline using bisection method to
// find points which discretize the curve to within [Spline3Sampler.Tol] error
// These points are then appended to dst and the result returned.
//
// It does not append points at extremes t=0 and t=1.
// maxDepth determines the max amount of times to subdivide the curve.
// The max amount of subdivisions (points appended) is given by 2**maxDepth.
func (s *Spline3Sampler) SampleBisect(dst []Vec, maxDepth int) []Vec {
	if maxDepth <= 0 {
		panic("invalid depth")
	} else if s.Tolerance < 0 {
		panic("negative tolerance")
	} else if s.Tolerance == 0 {
		panic("zero tolerance, initialize Spline3Sampler Tolerance field to a small value, i.e: 0.01")
	}
	baseRes := 1.0 / float32(uint(1)<<uint(maxDepth))
	return s.sampleBisect(dst, maxDepth, 0, s.Evaluate(0), 0, baseRes)
}

// SampleBisectWithExtremes is same as [Spline3Sampler.SampleBisect] but adding start and end points at t=0, t=1.
func (s *Spline3Sampler) SampleBisectWithExtremes(dst []Vec, maxDepth int) []Vec {
	if maxDepth <= 0 {
		panic("invalid depth")
	} else if s.Tolerance < 0 {
		panic("negative tolerance")
	} else if s.Tolerance == 0 {
		panic("zero tolerance, initialize Spline3Sampler Tolerance field to a small value, i.e: 0.01")
	}
	baseRes := 1.0 / float32(uint(1)<<uint(maxDepth))
	xStart := s.Evaluate(0)
	dst = append(dst, xStart)
	dst = s.sampleBisect(dst, maxDepth, 0, xStart, 0, baseRes)
	dst = append(dst, s.Evaluate(1))
	return dst
}

func (s *Spline3Sampler) sampleBisect(dst []Vec, lvl, idx int, xstart Vec, tstart, baseRes float32) []Vec {
	if lvl == 0 {
		if idx != 0 {
			dst = append(dst, xstart)
		}
		return dst
	}
	// Same algorithm as octree splitting but in 1D.
	slvl := lvl - 1
	midIdx := idx + 1<<slvl
	endIdx := idx + 1<<lvl

	tend := baseRes * float32(endIdx)
	tmid := baseRes * float32(midIdx)
	xend := s.Evaluate(tend)
	xmid := s.Evaluate(tmid)
	if Collinear(xstart, xmid, xend, s.Tolerance) {
		// Check offset- curve may be undersampled.
		var k float32 = 0.45
		tmid2 := tstart + k*(tend-tstart)
		xmid2 := s.Evaluate(tmid2)
		if Collinear(xstart, xmid2, xend, s.Tolerance) {
			if idx != 0 {
				dst = append(dst, xstart)
			}
			return dst // Won't subdivide further, this section of spline is straight.
		}
	}

	dst = s.sampleBisect(dst, slvl, idx, xstart, tstart, baseRes)
	dst = s.sampleBisect(dst, slvl, midIdx, xmid, tmid, baseRes)
	return dst
}

type vec4 struct {
	x, y, z, w float32
}

// matquatmul4 multiplies the 4x4 matrix m by the quaternion q treated as a
// column 4-vector (I, J, K, W) and returns the result.
func matvecmul4(m Mat4, q vec4) (res vec4) {
	res.x = m.x00*q.x + m.x01*q.y + m.x02*q.z + m.x03*q.w
	res.y = m.x10*q.x + m.x11*q.y + m.x12*q.z + m.x13*q.w
	res.z = m.x20*q.x + m.x21*q.y + m.x22*q.z + m.x23*q.w
	res.w = m.x30*q.x + m.x31*q.y + m.x32*q.z + m.x33*q.w
	return res
}
