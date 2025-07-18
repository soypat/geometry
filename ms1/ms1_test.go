package ms1

import (
	"math/rand"
	"testing"
)

func Test_gridSubdomainInternal(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < 100000; i++ {
		dmin := float32(rng.Float64())
		dmax := dmin + float32(rng.Float64())
		dsize := dmax - dmin

		smin := dmin + float32(rng.Float64())*dsize
		smaxsize := dmax - smin
		smax := smin + float32(rng.Float64())*smaxsize
		if i%8 == 0 {
			smax = dmax
		}
		if i%16 == 0 {
			smin = dmin
		}
		ssize := smax - smin
		for Nd := 2; Nd < 10; Nd++ {
			istart, nsub := GridSubdomain(dmin, dmax, Nd, smin, smax)
			if nsub > Nd {
				t.Error("subdomain contains more grid points than actual domain", nsub, Nd)
			} else if istart >= Nd {
				t.Error("istart >= Nd", istart, Nd)
			} else if istart+nsub > Nd {
				t.Error("istart+nsub>Nd", istart, nsub, istart+nsub, Nd)
			} else if nsub < 0 {
				t.Error("nsub<0", nsub)
			}
			dx := dsize / float32(Nd-1)
			calc_xmax := dmin + float32(istart+nsub-1)*dx
			calc_xmin := dmin + float32(istart)*dx
			if nsub > 0 && calc_xmax > smax {
				t.Error("xmax > smax", calc_xmax, smax)
			}
			if nsub > 0 && calc_xmin < smin {
				t.Error("xmin < smin", calc_xmin, smin)
			}
			if nsub > 0 && calc_xmax > dmax {
				t.Error("xmax > dmax", calc_xmax, dmax)
			}
			_ = ssize
			if t.Failed() {
				istart, nsub = GridSubdomain(dmin, dmax, Nd, smin, smax) // For debugging.
				t.Fatalf("failed on output: i=%d ns=%d  input: d=(%.3f,%.3f) N=%d sub=(%.3f, %.3f) ix=%.3f",
					istart, nsub, dmin, dmax, Nd, smin, smax, calc_xmin)
			}
		}
	}
}
