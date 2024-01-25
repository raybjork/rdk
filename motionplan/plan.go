package motionplan

import (
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	pb "go.viam.com/api/service/motion/v1"

	"go.viam.com/rdk/motionplan/ik"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Plan is an interface that describes plans returned by this package.  There are two key components to a Plan:
// Its Trajectory contains information pertaining to the commands required to actuate the robot to realize the Plan.
// Its Path contains information describing the Pose of the robot as it travels the Plan.
type Plan interface {
	Trajectory() Trajectory
	Path() Path
}

// RemainingPlan returns a new Plan equal to the given plan from the waypointIndex onwards.
func RemainingPlan(p Plan, waypointIndex int) (Plan, error) {
	plan, ok := p.(*rrtPlan)
	if !ok {
		return nil, errBadPlanImpl
	}
	if waypointIndex < 0 || waypointIndex > len(plan.traj) {
		return nil, fmt.Errorf("could not access trajectory index %d, must be between 0 and %d", waypointIndex, len(plan.traj))
	}
	return &rrtPlan{
		path:  plan.Path()[waypointIndex:],
		traj:  plan.Trajectory()[waypointIndex:],
		nodes: plan.nodes[waypointIndex:],
	}, nil
}

// OffsetPlan returns a new Plan that is equivalent to the given Plan if its Path was offset by the given Pose.
func OffsetPlan(p Plan, offset spatialmath.Pose) (Plan, error) {
	plan, ok := p.(*rrtPlan)
	if !ok {
		return nil, errBadPlanImpl
	}
	newPath := make([]PathStep, 0, len(plan.Path()))
	for _, step := range plan.Path() {
		newStep := make(PathStep, len(step))
		for frame, pose := range step {
			newStep[frame] = referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.Compose(offset, pose.Pose()))
		}
		newPath = append(newPath, newStep)
	}
	return &rrtPlan{
		path:  newPath,
		traj:  plan.traj,
		nodes: plan.nodes,
	}, nil
}

// NewGeoPlan returns a Plan containing GPS coordinates smuggled into the Pose struct. Each GPS point is created using:
// A Point with X as the longitude and Y as the latitude
// An orientation using the heading as the theta in an OrientationVector with Z=1.
func NewGeoPlan(p Plan, geoOrigin *spatialmath.GeoPose) (Plan, error) {
	plan, ok := p.(*rrtPlan)
	if !ok {
		return nil, errBadPlanImpl
	}
	newPath := make([]PathStep, 0, len(plan.Path()))
	for _, step := range plan.Path() {
		newStep := make(PathStep)
		for frame, pif := range step {
			pose := pif.Pose()
			geoPose := spatialmath.PoseToGeoPose(geoOrigin, pose)
			heading := math.Mod(math.Abs(geoPose.Heading()-360), 360)
			o := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: heading}
			smuggledGeoPose := spatialmath.NewPose(r3.Vector{X: geoPose.Location().Lng(), Y: geoPose.Location().Lat()}, o)
			newStep[frame] = referenceframe.NewPoseInFrame(pif.Parent(), smuggledGeoPose)
		}
		newPath = append(newPath, newStep)
	}
	return &rrtPlan{
		traj:  plan.traj,
		path:  newPath,
		nodes: plan.nodes,
	}, nil
}

// Trajectory is a slice of maps describing a series of Inputs for a robot to travel to in the course of following a Plan.
// Each item in this slice maps a Frame to the Inputs that Frame should be modified by.
type Trajectory []map[string][]referenceframe.Input

// GetFrameInputs is a helper function which will extract the waypoints of a single frame from the map output of a trajectory.
func (traj Trajectory) GetFrameInputs(frameName string) ([][]referenceframe.Input, error) {
	solution := make([][]referenceframe.Input, 0, len(traj))
	for _, step := range traj {
		frameStep, ok := step[frameName]
		if !ok {
			return nil, fmt.Errorf("frame named %s not found in trajectory", frameName)
		}
		solution = append(solution, frameStep)
	}
	return solution, nil
}

// String returns a human-readable version of the trajectory, suitable for debugging.
func (traj Trajectory) String() string {
	var str string
	for _, step := range traj {
		str += "\n"
		for frame, input := range step {
			if len(input) > 0 {
				str += fmt.Sprintf("%s: %v\t", frame, input)
			}
		}
	}
	return str
}

// EvaluateCost calculates a cost to a trajectory as measured by the given distFunc Metric.
func (traj Trajectory) EvaluateCost(distFunc ik.SegmentMetric) (totalCost float64) {
	last := map[string][]referenceframe.Input{}
	for _, step := range traj {
		for frame, inputs := range step {
			if len(inputs) > 0 {
				if lastInputs, ok := last[frame]; ok {
					cost := distFunc(&ik.Segment{StartConfiguration: lastInputs, EndConfiguration: inputs})
					totalCost += cost
				}
				last[frame] = inputs
			}
		}
	}
	return totalCost
}

// Path is a slice of PathSteps describing a series of Poses for a robot to travel to in the course of following a Plan.
type Path []PathStep

func newRelativePath(solution []node, sf *solverFrame) (Path, error) {
	path := make(Path, 0, len(solution))
	for _, step := range solution {
		stepMap := sf.sliceToMap(step.Q())
		step := make(map[string]*referenceframe.PoseInFrame)
		for frame := range stepMap {
			tf, err := sf.fss.Transform(stepMap, referenceframe.NewPoseInFrame(frame, spatialmath.NewZeroPose()), referenceframe.World)
			if err != nil {
				return nil, err
			}
			pose, ok := tf.(*referenceframe.PoseInFrame)
			if !ok {
				return nil, errors.New("pose not transformable")
			}
			step[frame] = pose
		}
		path = append(path, step)
	}
	return path, nil
}

func newAbsolutePathFromRelative(path Path) (Path, error) {
	if len(path) < 2 {
		return nil, errors.New("need to have at least 2 elements in path")
	}
	newPath := make([]PathStep, 0, len(path))
	newPath = append(newPath, path[0])
	for i, step := range path[1:] {
		newStep := make(PathStep, len(step))
		for frame, pose := range step {
			newStep[frame] = referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.Compose(newPath[i][frame].Pose(), pose.Pose()))
		}
		newPath = append(newPath, newStep)
	}
	return newPath, nil
}

// GetFramePoses returns a slice of poses a given frame should visit in the course of the Path.
func (path Path) GetFramePoses(frameName string) ([]spatialmath.Pose, error) {
	poses := []spatialmath.Pose{}
	for _, step := range path {
		pose, ok := step[frameName]
		if !ok {
			return nil, fmt.Errorf("frame named %s not found in path", frameName)
		}
		poses = append(poses, pose.Pose())
	}
	return poses, nil
}

func (path Path) String() string {
	var str string
	for _, step := range path {
		str += "\n"
		for frame, pose := range step {
			str += fmt.Sprintf("%s: %v\t", frame, pose.Pose().Point())
		}
	}
	return str
}

// PathStep is a mapping of Frame names to PoseInFrames.
// TODO: If the frame system ever uses resource names instead of strings this should be adjusted too.
type PathStep map[string]*referenceframe.PoseInFrame

// ToProto converts a PathStep to its representation in protobuf.
func (ps PathStep) ToProto() *pb.PlanStep {
	step := make(map[string]*pb.ComponentState)
	for name, pose := range ps {
		pbPose := spatialmath.PoseToProtobuf(pose.Pose())
		step[name] = &pb.ComponentState{Pose: pbPose}
	}
	return &pb.PlanStep{Step: step}
}

// PathStepFromProto converts a *pb.PlanStep to a PlanStep.
func PathStepFromProto(ps *pb.PlanStep) (PathStep, error) {
	if ps == nil {
		return PathStep{}, errors.New("received nil *pb.PlanStep")
	}

	step := make(PathStep, len(ps.Step))
	for k, v := range ps.Step {
		step[k] = referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPoseFromProtobuf(v.Pose))
	}
	return step, nil
}