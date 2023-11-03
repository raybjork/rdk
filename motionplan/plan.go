package motionplan

import (
	"fmt"
	"math"

	"go.viam.com/rdk/motionplan/ik"
	"go.viam.com/rdk/referenceframe"
)

type Plan struct {
	Path() Path
	Trajectory() Trajectory
	*rrtMaps
}

// Path describes a motion plan.
type Path []map[string][]referenceframe.Input

// GetFrameSteps is a helper function which will extract the waypoints of a single frame from the map output of a robot path.
func (plan Path) GetFrameSteps(frameName string) ([][]referenceframe.Input, error) {
	solution := make([][]referenceframe.Input, 0, len(plan))
	for _, step := range plan {
		frameStep, ok := step[frameName]
		if !ok {
			return nil, fmt.Errorf("frame named %s not found in solved motion plan", frameName)
		}
		solution = append(solution, frameStep)
	}
	return solution, nil
}

// String returns a human-readable version of the Plan, suitable for debugging.
func (plan Path) String() string {
	var str string
	for _, step := range plan {
		str += "\n"
		for component, input := range step {
			if len(input) > 0 {
				str += fmt.Sprintf("%s: %v\t", component, input)
			}
		}
	}
	return str
}

// Evaluate assigns a numeric score to a plan that corresponds to the cumulative distance between input waypoints in the plan.
func (plan Path) Evaluate(distFunc ik.SegmentMetric) (totalCost float64) {
	if len(plan) < 2 {
		return math.Inf(1)
	}
	last := map[string][]referenceframe.Input{}
	for _, step := range plan {
		for component, inputs := range step {
			if len(inputs) > 0 {
				if lastInputs, ok := last[component]; ok {
					cost := distFunc(&ik.Segment{StartConfiguration: lastInputs, EndConfiguration: inputs})
					totalCost += cost
				}
				last[component] = inputs
			}
		}
	}
	return totalCost
}
