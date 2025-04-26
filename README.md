# geometry
[![go.dev reference](https://pkg.go.dev/badge/github.com/soypat/geometry)](https://pkg.go.dev/github.com/soypat/geometry)
[![Go Report Card](https://goreportcard.com/badge/github.com/soypat/geometry)](https://goreportcard.com/report/github.com/soypat/geometry)
[![codecov](https://codecov.io/gh/soypat/geometry/branch/main/graph/badge.svg)](https://codecov.io/gh/soypat/geometry)
[![Go](https://github.com/soypat/geometry/actions/workflows/go.yml/badge.svg)](https://github.com/soypat/geometry/actions/workflows/go.yml)
[![sourcegraph](https://sourcegraph.com/github.com/soypat/geometry/-/badge.svg)](https://sourcegraph.com/github.com/soypat/geometry?badge)

Stackful, correct, lean and performant library ideal for any use case. From embedded systems to GPU usage.
 
## Features
- Vector and matrices that map to GPU alignment
- Quaternions
- 2D/3D Grid generation and traversal
- Heapless 3D Octree implementation
    - Is stupid fast.
- Performant 3x3 SVD and QR decomposition
- 2D/3D Triangles
    - Closest point to a triangle algorithm
- Tetrahedrons!
- Bounding boxes
- Polygon generation with arc and chamfering
- 2D splines with support for Quadratic and cubic modes
    - Provided splines are: Cubic/quadratic Bezier, Hermite spline, Basis spline, Cardinal spline, Catmull-Rom spline 
- 2D/3D Basic geometries like Line, Plane and their algorithms
- Few 1D math conveniences

## Module structure
- ms3..ms1 contain 32-bit (`float32`) spatial geometrical primitives.
- md3..md1 contain 64-bit (`float64`) spatial geometrical primitive. This code is identically duplicated from ms* packages using code generation, including tests.

## Development
Code developed is exclusively `float32`. `float64` code is generated automatically from the `float32` code by running `gen.go`.

The `internal` package serves as a place to store data that is dependent on whether the implementation is 64 or 32 bit.