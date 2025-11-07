package i3

// Cube implements a tree cube for Octree algorithms.
type Cube struct {
	// Vec stores the shifted
	Vec
	// Level keeps track of the level in the tree.
	//  - Level==1 means the cube is the smallest possible cube.
	//  - Level==0 is an invalid level. May be used as a flag to signal the cube has been discarded or processed and ready for discard.
	Level int
}

// IsSmallest returns true if Lvl==1. This means the cube cannot be decomposed further with [Cube.Octree].
func (c Cube) IsSmallest() bool { return c.Level == 1 }

// IsSecondSmallest returns true if Lvl==2. This means the cube can be decomposed once more with [Cube.Octree].
func (c Cube) IsSecondSmallest() bool { return c.Level == 2 }

// DecomposesTo returns the amount of cubes generated from decomposing the cube down to cubes of the argument target level.
func (c Cube) DecomposesTo(targetLevel int) uint64 {
	if targetLevel > c.Level {
		panic("invalid targetLvl to icube.decomposesTo")
	}
	return Pow8(c.Level - targetLevel)
}

// Size returns the length of one of the icube's sides.
func (c Cube) Size() (resUnits int) {
	return 1 << (c.Level - 1)
}

// Supercube returns the ICube3's parent octree ICube3.
func (c Cube) Supercube() Cube {
	upLevel := c.Level + 1
	bitmask := (1 << upLevel) - 1
	return Cube{
		Vec:   c.Vec.AndnotScalar(bitmask),
		Level: upLevel,
	}
}

// Index returns the indices corresponding to the ICube3 in the root cube.
// By multiplying the resulting indices by the smallest cube size one can obtain the origin of the ICube in space.
func (c Cube) Index() Vec {
	return c.Vec.ShiftRightScalar(c.Level) // icube indices per level in the octree.
}

// Octree returns the 8 sub-cubes of the receiver.
func (c Cube) Octree() [8]Cube {
	level := c.Level - 1
	if level <= 0 {
		panic("invalid operation: octree for level<=1")
	}
	s := 1 << level
	return [8]Cube{
		{Vec: c.Add(Vec{0, 0, 0}), Level: level},
		{Vec: c.Add(Vec{s, 0, 0}), Level: level},
		{Vec: c.Add(Vec{s, s, 0}), Level: level},
		{Vec: c.Add(Vec{0, s, 0}), Level: level},
		{Vec: c.Add(Vec{0, 0, s}), Level: level},
		{Vec: c.Add(Vec{s, 0, s}), Level: level},
		{Vec: c.Add(Vec{s, s, s}), Level: level},
		{Vec: c.Add(Vec{0, s, s}), Level: level},
	}
}

// Pow8 returns 8**y.
func Pow8(y int) uint64 {
	if y < len(_pow8) {
		return _pow8[y]
	}
	panic("overflow Pow8")
}

// Pow4 returns 4**y.
func Pow4(y int) uint64 {
	if y < len(_pow4) {
		return _pow4[y]
	}
	panic("overflow Pow4")
}

var _pow8 = [...]uint64{
	0:  1,
	1:  8,
	2:  8 * 8,
	3:  8 * 8 * 8,
	4:  8 * 8 * 8 * 8,
	5:  8 * 8 * 8 * 8 * 8,
	6:  8 * 8 * 8 * 8 * 8 * 8,
	7:  8 * 8 * 8 * 8 * 8 * 8 * 8,
	8:  8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	9:  8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	10: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	11: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	12: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	13: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	14: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	15: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	16: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	17: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	18: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	19: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	20: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	21: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8,
	// 22: 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8 * 8, // overflows
}

var _pow4 = [...]uint64{
	0:  1,
	1:  4,
	2:  4 * 4,
	3:  4 * 4 * 4,
	4:  4 * 4 * 4 * 4,
	5:  4 * 4 * 4 * 4 * 4,
	6:  4 * 4 * 4 * 4 * 4 * 4,
	7:  4 * 4 * 4 * 4 * 4 * 4 * 4,
	8:  4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	9:  4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	10: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	11: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	12: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	13: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	14: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	15: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	16: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	17: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	18: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	19: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	20: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	21: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	22: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	23: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	24: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	25: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	26: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	27: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	28: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	29: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	30: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4,
	// 31: 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4 * 4, // overflows
}
