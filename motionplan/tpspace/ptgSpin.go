package tpspace

import "math"

// ptgDiffDriveSpin defines a PTG family composed of a spinning trajectory
type ptgDiffDriveSpin struct {
	maxMMPS float64 // millimeters per second velocity to target
	maxRPS  float64 // radians per second of rotation when driving at maxMMPS and turning at max turning radius
}

// NewCirclePTG creates a new PrecomputePTG of type ptgDiffDriveSpin.
func NewSpinPTG(maxMMPS, maxRPS, k float64) PrecomputePTG {
	return &ptgDiffDriveC{
		maxMMPS: maxMMPS,
		maxRPS:  maxRPS,
	}
}

// For this particular driver, turn at the max rotation in the direction specified
func (ptg *ptgDiffDriveSpin) PTGVelocities(alpha, dist float64) (float64, float64, error) {
	// rotate this much before going straight
	turnDist := math.Abs(alpha) / ptg.maxRPS

	v := ptg.maxMMPS
	w := 0.

	if dist < turnDist {
		v = 0
		w = math.Copysign(ptg.maxRPS, alpha)
	}
	return v, w, nil
}
