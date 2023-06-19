// Package generic contains a gRPC based generic service serviceServer.
package generic

import (
	"context"

	commonpb "go.viam.com/api/common/v1"
	genericpb "go.viam.com/api/component/generic/v1"

	rdkutils "go.viam.com/rdk/protoutils"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
)

// serviceServer implements the resource.Generic service.
type serviceServer struct {
	genericpb.UnimplementedGenericServiceServer
	coll resource.APIResourceCollection[resource.Resource]
}

// NewRPCServiceServer constructs an generic gRPC service serviceServer.
func NewRPCServiceServer(coll resource.APIResourceCollection[resource.Resource]) interface{} {
	return &serviceServer{coll: coll}
}

// DoCommand receives arbitrary commands.
func (s *serviceServer) DoCommand(ctx context.Context, req *commonpb.DoCommandRequest) (*commonpb.DoCommandResponse, error) {
	res, err := s.coll.Resource(req.GetName())
	if err != nil {
		return nil, err
	}
	return rdkutils.DoFromResourceServer(ctx, res, req)
}

func (s *serviceServer) Geometries(ctx context.Context, req *commonpb.GetGeometriesRequest) (*commonpb.GetGeometriesResponse, error) {
	res, err := s.coll.Resource(req.GetName())
	if err != nil {
		return nil, err
	}
	// although Geometries is a function on a generic component it is not necessary for all resources to implement so need assertion here.
	shaped, err := utils.AssertType[resource.Shaped](res)
	if err != nil {
		return nil, err
	}
	return rdkutils.GeometriesFromResourceServer(ctx, shaped, req)
}
