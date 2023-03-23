package wheeled

import (
	"bytes"
	"context"
	"math"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

type kinematicWheeledBase struct {
	*wheeledBase
	slam slam.Service
	// TODO(RSDK-2393): When SLAM services know their own name this can be taken out
	slamName string
	model    referenceframe.Model
}

// WrapWithKinematics takes a wheeledBase component and adds a slam service to it
// It also adds kinematic model so that it can be controlled.
func WrapWithKinematics(ctx context.Context, base *wheeledBase, slamName string, slam slam.Service) (base.KinematicBase, error) {
	var err error
	kwb := &kinematicWheeledBase{
		wheeledBase: base,
		slam:        slam,
		slamName:    slamName,
	}
	kwb.model, err = kwb.buildModel(ctx)
	if err != nil {
		return nil, err
	}
	return kwb, err
}

func (kwb *kinematicWheeledBase) ModelFrame() referenceframe.Model {
	return kwb.model
}

func (kwb *kinematicWheeledBase) currentPose(ctx context.Context) (spatialmath.Pose, error) {
	// TODO: make a transformation from the component reference to the base frame
	pose, _, err := kwb.slam.GetPosition(ctx, kwb.slamName)
	return pose, err
}

func (kwb *kinematicWheeledBase) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	// TODO: make a transformation from the component reference to the base frame
	pose, err := kwb.currentPose(ctx)
	if err != nil {
		return nil, err
	}
	pt := pose.Point()

	// Need to get X, Z from lidar because Y points down
	// TODO: make a ticket to give rplidar kinematic information so that you don't have to do this here
	return []referenceframe.Input{{Value: pt.X}, {Value: pt.Z}}, nil
}

func (kwb *kinematicWheeledBase) GoToInputs(ctx context.Context, goal []referenceframe.Input) error {
	// TODO: may want to save the startPose separately
	// TODO: code janitor
	currentPose, err := kwb.currentPose(ctx)
	if err != nil {
		return err
	}
	currentPt := currentPose.Point()
	desiredHeading := math.Atan2(currentPt.Z-goal[1].Value, currentPt.X-goal[0].Value)
	distance := math.Hypot(currentPt.Z-goal[1].Value, currentPt.X-goal[0].Value)


	// TODO: we do want the pitch here but this is domain limited to -90 to 90, need math to fix this
	heading := utils.RadToDeg(currentPose.Orientation().EulerAngles().Pitch)
	// While base is not at the goal
	// TO DO figure out sane threshold.
	for distance > 5 {
		// If heading is ok, go forward
		// Otherwise spin until base is heading correct way
		// TODO make a threshold
		if math.Abs(heading-desiredHeading) > 0 {
			// TODO (rh) create context with cancel
			// TODO use a speed that is not garbage
			if err := kwb.Spin(ctx, heading-desiredHeading, 10, nil); err != nil {
				return err
			}
		} else {
			// TODO check if we are in mm in SLAM and multiply by 1000 if so
			distance := math.Hypot(currentPt.Z-goal[1].Value, currentPt.X-goal[0].Value)
			if err := kwb.MoveStraight(ctx, int(distance), 10, nil); err != nil {
				return err
			}

		}

		// Calculate current state
		currentPose, err = kwb.currentPose(ctx)
		if err != nil {
			return err
		}
		currentPt = currentPose.Point()
		heading = utils.RadToDeg(currentPose.Orientation().EulerAngles().Pitch)
		distance = math.Hypot(currentPt.Z-goal[1].Value, currentPt.X-goal[0].Value)

	}
	return nil
}

// ModelFrame builds the kinematic model associated with the kinematicWheeledBase
// Note that this model is not intended to be registered in the frame system.
func (kwb *kinematicWheeledBase) buildModel(ctx context.Context) (referenceframe.Model, error) {
	if kwb.collisionGeometry == nil {
		return nil, errors.New("cannot create model for base with no collision geometry")
	}

	// get the limits of the SLAM map to set as the extents of the frame
	// TODO(RSDK-2393): figure out how to get the slam name here to make the proper call to this method
	data, err := slam.GetPointCloudMapFull(ctx, kwb.slam, kwb.slamName)
	if err != nil {
		return nil, err
	}
	dims, err := pointcloud.GetPCDMetaData(bytes.NewReader(data))
	if err != nil {
		return nil, err
	}

	// build the model - SLAM convention is that the XZ plane is the ground plane
	frame2D, err := referenceframe.NewMobile2DFrame(
		kwb.collisionGeometry.Label(),
		[]referenceframe.Limit{{Min: dims.MinX, Max: dims.MaxX}, {Min: dims.MinZ, Max: dims.MaxZ}},
		kwb.collisionGeometry)
	if err != nil {
		return nil, err
	}
	model := referenceframe.NewSimpleModel(kwb.name)
	model.OrdTransforms = []referenceframe.Frame{frame2D}
	return model, nil
}

func collisionGeometry(cfg config.Component) (spatialmath.Geometry, error) {
	// TODO(rb): this is a hacky workaround for not having kinematics for bases yet
	// we create a sphere that would encompass the config geometry's rotation a full 360 degrees
	// TODO(RSDK-1014): the orientation of this model will matter for collision checking,
	// and should match the convention of +Y being forward for bases
	if cfg.Frame == nil || cfg.Frame.Geometry == nil {
		return nil, errors.New("base not configured with a geometry on its frame, cannot create collision geometry for it")
	}
	geoCfg := cfg.Frame.Geometry
	r := geoCfg.TranslationOffset.Norm()
	switch geoCfg.Type {
	case spatialmath.BoxType:
		r += r3.Vector{X: geoCfg.X, Y: geoCfg.Y, Z: geoCfg.Z}.Norm() / 2
	case spatialmath.SphereType:
		r += geoCfg.R
	case spatialmath.CapsuleType:
		r += geoCfg.L / 2
	case spatialmath.UnknownType:
		// no type specified, iterate through supported types and try to infer intent
		if norm := (r3.Vector{X: geoCfg.X, Y: geoCfg.Y, Z: geoCfg.Z}).Norm(); norm > 0 {
			r += norm / 2
		} else if geoCfg.L != 0 {
			r += geoCfg.L / 2
		} else {
			r += geoCfg.R
		}
	case spatialmath.PointType:
	default:
		return nil, spatialmath.ErrGeometryTypeUnsupported
	}
	sphere, err := spatialmath.NewSphere(spatialmath.NewZeroPose(), r, geoCfg.Label)
	if err != nil {
		return nil, err
	}
	return sphere, nil
}
