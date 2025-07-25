// Package inject provides dependency injected structures for mocking interfaces.
package inject

import (
	"context"
	"errors"
	"sync"
	"time"

	"github.com/google/uuid"
	"go.viam.com/utils/pexec"

	"go.viam.com/rdk/cloud"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/robot/packages"
	"go.viam.com/rdk/session"
)

// Robot is an injected robot.
type Robot struct {
	robot.LocalRobot
	Mu                       sync.RWMutex // Ugly, has to be manually locked if a test means to swap funcs on an in-use robot.
	GetModelsFromModulesFunc func(ctx context.Context) ([]resource.ModuleModel, error)
	RemoteByNameFunc         func(name string) (robot.Robot, bool)
	ResourceByNameFunc       func(name resource.Name) (resource.Resource, error)
	RemoteNamesFunc          func() []string
	ResourceNamesFunc        func() []resource.Name
	ResourceRPCAPIsFunc      func() []resource.RPCAPI
	ProcessManagerFunc       func() pexec.ProcessManager
	ConfigFunc               func() *config.Config
	LoggerFunc               func() logging.Logger
	CloseFunc                func(ctx context.Context) error
	StopAllFunc              func(ctx context.Context, extra map[resource.Name]map[string]interface{}) error
	FrameSystemConfigFunc    func(ctx context.Context) (*framesystem.Config, error)
	TransformPoseFunc        func(
		ctx context.Context,
		pose *referenceframe.PoseInFrame,
		dst string,
		additionalTransforms []*referenceframe.LinkInFrame,
	) (*referenceframe.PoseInFrame, error)
	TransformPointCloudFunc func(ctx context.Context, srcpc pointcloud.PointCloud, srcName, dstName string) (pointcloud.PointCloud, error)
	CurrentInputsFunc       func(ctx context.Context) (referenceframe.FrameSystemInputs, error)
	ModuleAddressesFunc     func() (config.ParentSockAddrs, error)
	CloudMetadataFunc       func(ctx context.Context) (cloud.Metadata, error)
	MachineStatusFunc       func(ctx context.Context) (robot.MachineStatus, error)
	ShutdownFunc            func(ctx context.Context) error
	ListTunnelsFunc         func(ctx context.Context) ([]config.TrafficTunnelEndpoint, error)

	ops        *operation.Manager
	SessMgr    session.Manager
	PackageMgr packages.Manager
}

// MockResourcesFromMap mocks ResourceNames and ResourceByName based on a resource map.
func (r *Robot) MockResourcesFromMap(rs map[resource.Name]resource.Resource) {
	func() {
		r.Mu.Lock()
		defer r.Mu.Unlock()
		r.ResourceNamesFunc = func() []resource.Name {
			result := []resource.Name{}
			for name := range rs {
				result = append(result, name)
			}
			return result
		}
	}()
	func() {
		r.Mu.Lock()
		defer r.Mu.Unlock()
		r.ResourceByNameFunc = func(name resource.Name) (resource.Resource, error) {
			result, ok := rs[name]
			if ok {
				return result, nil
			}
			return nil, errors.New("not found")
		}
	}()
}

// RemoteByName calls the injected RemoteByName or the real version.
func (r *Robot) RemoteByName(name string) (robot.Robot, bool) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.RemoteByNameFunc == nil {
		return r.LocalRobot.RemoteByName(name)
	}
	return r.RemoteByNameFunc(name)
}

// ResourceByName calls the injected ResourceByName or the real version.
func (r *Robot) ResourceByName(name resource.Name) (resource.Resource, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ResourceByNameFunc == nil {
		return r.LocalRobot.ResourceByName(name)
	}
	return r.ResourceByNameFunc(name)
}

// RemoteNames calls the injected RemoteNames or the real version.
func (r *Robot) RemoteNames() []string {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.RemoteNamesFunc == nil {
		return r.LocalRobot.RemoteNames()
	}
	return r.RemoteNamesFunc()
}

// ResourceNames calls the injected ResourceNames or the real version.
func (r *Robot) ResourceNames() []resource.Name {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ResourceNamesFunc == nil {
		return r.LocalRobot.ResourceNames()
	}
	return r.ResourceNamesFunc()
}

// ResourceRPCAPIs returns a list of all known resource RPC APIs.
func (r *Robot) ResourceRPCAPIs() []resource.RPCAPI {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ResourceRPCAPIsFunc == nil {
		return r.LocalRobot.ResourceRPCAPIs()
	}
	return r.ResourceRPCAPIsFunc()
}

// OperationManager calls the injected OperationManager or the real version.
func (r *Robot) OperationManager() *operation.Manager {
	r.Mu.RLock()
	defer r.Mu.RUnlock()

	if r.ops == nil {
		r.ops = operation.NewManager(r.Logger())
	}
	return r.ops
}

// SessionManager calls the injected SessionManager or the real version.
func (r *Robot) SessionManager() session.Manager {
	r.Mu.RLock()
	defer r.Mu.RUnlock()

	if r.SessMgr == nil {
		return noopSessionManager{}
	}
	return r.SessMgr
}

// PackageManager calls the injected PackageManager or the real version.
func (r *Robot) PackageManager() packages.Manager {
	r.Mu.RLock()
	defer r.Mu.RUnlock()

	if r.PackageMgr == nil {
		return packages.NewNoopManager()
	}
	return r.PackageMgr
}

// Config calls the injected Config or the real version.
func (r *Robot) Config() *config.Config {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ConfigFunc == nil {
		return r.LocalRobot.Config()
	}
	return r.ConfigFunc()
}

// Logger calls the injected Logger or the real version.
func (r *Robot) Logger() logging.Logger {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.LoggerFunc == nil {
		return r.LocalRobot.Logger()
	}
	return r.LoggerFunc()
}

// Close calls the injected Close or the real version.
func (r *Robot) Close(ctx context.Context) error {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.CloseFunc == nil {
		if r.LocalRobot == nil {
			return nil
		}
		return r.LocalRobot.Close(ctx)
	}
	return r.CloseFunc(ctx)
}

// StopAll calls the injected StopAll or the real version.
func (r *Robot) StopAll(ctx context.Context, extra map[resource.Name]map[string]interface{}) error {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.StopAllFunc == nil {
		return r.LocalRobot.StopAll(ctx, extra)
	}
	return r.StopAllFunc(ctx, extra)
}

// GetModelsFromModules calls the injected GetModelsFromModules or the real one.
func (r *Robot) GetModelsFromModules(ctx context.Context) ([]resource.ModuleModel, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.GetModelsFromModulesFunc == nil {
		return r.LocalRobot.GetModelsFromModules(ctx)
	}
	return r.GetModelsFromModulesFunc(ctx)
}

// FrameSystemConfig calls the injected FrameSystemConfig or the real version.
func (r *Robot) FrameSystemConfig(ctx context.Context) (*framesystem.Config, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.FrameSystemConfigFunc == nil {
		return r.LocalRobot.FrameSystemConfig(ctx)
	}

	return r.FrameSystemConfigFunc(ctx)
}

// TransformPose calls the injected TransformPose or the real version.
func (r *Robot) TransformPose(
	ctx context.Context,
	pose *referenceframe.PoseInFrame,
	dst string,
	additionalTransforms []*referenceframe.LinkInFrame,
) (*referenceframe.PoseInFrame, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.TransformPoseFunc == nil {
		return r.LocalRobot.TransformPose(ctx, pose, dst, additionalTransforms)
	}
	return r.TransformPoseFunc(ctx, pose, dst, additionalTransforms)
}

// TransformPointCloud calls the injected TransformPointCloud or the real version.
func (r *Robot) TransformPointCloud(ctx context.Context, srcpc pointcloud.PointCloud, srcName, dstName string,
) (pointcloud.PointCloud, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.TransformPointCloudFunc == nil {
		return r.LocalRobot.TransformPointCloud(ctx, srcpc, srcName, dstName)
	}
	return r.TransformPointCloudFunc(ctx, srcpc, srcName, dstName)
}

// CurrentInputs returns a map of the current inputs for each component of a machine's frame system
// and a map of statuses indicating which of the machine's components may be actuated through input values.
func (r *Robot) CurrentInputs(ctx context.Context) (referenceframe.FrameSystemInputs, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.CurrentInputsFunc == nil {
		return r.LocalRobot.CurrentInputs(ctx)
	}
	return r.CurrentInputs(ctx)
}

// ModuleAddresses calls the injected ModuleAddresses or the real one.
func (r *Robot) ModuleAddresses() (config.ParentSockAddrs, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ModuleAddressesFunc == nil {
		return r.LocalRobot.ModuleAddresses()
	}
	return r.ModuleAddressesFunc()
}

// CloudMetadata calls the injected CloudMetadata or the real one.
func (r *Robot) CloudMetadata(ctx context.Context) (cloud.Metadata, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.CloudMetadataFunc == nil {
		return r.LocalRobot.CloudMetadata(ctx)
	}
	return r.CloudMetadataFunc(ctx)
}

// MachineStatus calls the injected MachineStatus or the real one.
func (r *Robot) MachineStatus(ctx context.Context) (robot.MachineStatus, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.MachineStatusFunc == nil {
		return r.LocalRobot.MachineStatus(ctx)
	}
	return r.MachineStatusFunc(ctx)
}

// Shutdown calls the injected Shutdown or the real one.
func (r *Robot) Shutdown(ctx context.Context) error {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ShutdownFunc == nil {
		return r.LocalRobot.Shutdown(ctx)
	}
	return r.ShutdownFunc(ctx)
}

// ListTunnels calls the injected ListTunnels or the real one.
func (r *Robot) ListTunnels(ctx context.Context) ([]config.TrafficTunnelEndpoint, error) {
	r.Mu.RLock()
	defer r.Mu.RUnlock()
	if r.ListTunnelsFunc == nil {
		return r.LocalRobot.ListTunnels(ctx)
	}
	return r.ListTunnelsFunc(ctx)
}

type noopSessionManager struct{}

func (m noopSessionManager) Start(ctx context.Context, ownerID string) (*session.Session, error) {
	return session.New(ctx, ownerID, time.Minute, nil), nil
}

func (m noopSessionManager) All() []*session.Session {
	return nil
}

func (m noopSessionManager) FindByID(ctx context.Context, id uuid.UUID, ownerID string) (*session.Session, error) {
	return nil, session.ErrNoSession
}

func (m noopSessionManager) AssociateResource(id uuid.UUID, resourceName resource.Name) {
}

func (m noopSessionManager) Close() {
}

func (m noopSessionManager) ServerInterceptors() session.ServerInterceptors {
	return session.ServerInterceptors{}
}
