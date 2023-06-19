// Package generic contains a gRPC based generic client.
package generic

import (
	"context"

	"github.com/edaniels/golog"
	genericpb "go.viam.com/api/component/generic/v1"
	rdkutils "go.viam.com/rdk/protoutils"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"

	"go.viam.com/rdk/resource"
)

// client implements GenericServiceClient.
type client struct {
	resource.Named
	resource.TriviallyReconfigurable
	resource.TriviallyCloseable
	name   string
	client genericpb.GenericServiceClient
	logger golog.Logger
}

// NewClientFromConn constructs a new Client from connection passed in.
func NewClientFromConn(
	ctx context.Context,
	conn rpc.ClientConn,
	remoteName string,
	name resource.Name,
	logger golog.Logger,
) (resource.Resource, error) {
	c := genericpb.NewGenericServiceClient(conn)
	return &client{
		Named:  name.PrependRemote(remoteName).AsNamed(),
		name:   name.ShortName(),
		client: c,
		logger: logger,
	}, nil
}

func (c *client) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return rdkutils.DoFromResourceClient(ctx, c.client, c.name, cmd)
}

func (c *client) Geometries(ctx context.Context) ([]spatialmath.Geometry, error) {
	return rdkutils.GeometriesFromResourceClient(ctx, c.client, c.name)
}
