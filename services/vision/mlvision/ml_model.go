// Package mlvision uses an underlying model from the ML model service as a vision model,
// and wraps the ML model with the vision service methods.
package mlvision

import (
	"bufio"
	"context"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"

	"github.com/pkg/errors"
	"go.opencensus.io/trace"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/services/mlmodel"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/utils"
)

var model = resource.DefaultModelFamily.WithModel("mlmodel")

const (
	// UInt8 is one of the possible input/output types for tensors.
	UInt8 = "uint8"
	// Float32 is one of the possible input/output types for tensors.
	Float32 = "float32"
	// DefaultOutTensorName is the prefix key given to output tensors in the map
	// if there is no metadata. (output0, output1, etc.)
	DefaultOutTensorName = "output"
)

func init() {
	resource.RegisterService(vision.API, model, resource.Registration[vision.Service, *MLModelConfig]{
		DeprecatedRobotConstructor: func(
			ctx context.Context, r any, c resource.Config, logger logging.Logger,
		) (vision.Service, error) {
			attrs, err := resource.NativeConfig[*MLModelConfig](c)
			if err != nil {
				return nil, err
			}
			actualR, err := utils.AssertType[robot.Robot](r)
			if err != nil {
				return nil, err
			}
			return registerMLModelVisionService(ctx, c.ResourceName(), attrs, actualR, logger)
		},
	})
}

// MLModelConfig specifies the parameters needed to turn an ML model into a vision Model.
type MLModelConfig struct {
	ModelName        string            `json:"mlmodel_name"`
	RemapInputNames  map[string]string `json:"remap_input_names"`
	RemapOutputNames map[string]string `json:"remap_output_names"`
	BoxOrder         []int             `json:"xmin_ymin_xmax_ymax_order"`
	// optional parameter used to normalize the input image if the ML Model expects it
	MeanValue []float32 `json:"input_image_mean_value"`
	// optional parameter used to normalize the input image if the ML Model expects it
	StdDev []float32 `json:"input_image_std_dev"`
	// optional parameter used to change the input image to BGR format if the ML Model expects it
	IsBGR              bool               `json:"input_image_bgr"`
	DefaultConfidence  float64            `json:"default_minimum_confidence"`
	LabelConfidenceMap map[string]float64 `json:"label_confidences"`
	LabelPath          string             `json:"label_path"`
	DefaultCamera      string             `json:"camera_name"`
}

// Validate will add the ModelName as an implicit dependency to the robot.
func (conf *MLModelConfig) Validate(path string) ([]string, []string, error) {
	if conf.ModelName == "" {
		return nil, nil, errors.New("mlmodel_name cannot be empty")
	}
	if conf.LabelPath != "" {
		_, err := os.Stat(conf.LabelPath)
		if err != nil {
			return nil, nil, fmt.Errorf("failed to read file %s: %w", conf.LabelPath, err)
		}
	}
	if len(conf.MeanValue) != 0 {
		if len(conf.MeanValue) < 3 {
			return nil, nil, errors.New("input_image_mean_value attribute must have at least 3 values, one for each color channel")
		}
	}
	if len(conf.StdDev) != 0 {
		if len(conf.StdDev) < 3 {
			return nil, nil, errors.New("input_image_std_dev attribute must have at least 3 values, one for each color channel")
		}
	}
	for _, v := range conf.StdDev {
		if v == 0.0 {
			return nil, nil, errors.New("input_image_std_dev is not allowed to have 0 values, will cause division by 0")
		}
	}
	return []string{conf.ModelName}, nil, nil
}

func registerMLModelVisionService(
	ctx context.Context,
	name resource.Name,
	params *MLModelConfig,
	r robot.Robot,
	logger logging.Logger,
) (vision.Service, error) {
	_, span := trace.StartSpan(ctx, "service::vision::registerMLModelVisionService")
	defer span.End()

	mlm, err := mlmodel.FromRobot(r, params.ModelName)
	if err != nil {
		return nil, err
	}

	// the Maps that associates the tensor names as they are found in the model, to
	// what the vision service expects.
	inNameMap := &sync.Map{}
	for oldName, newName := range params.RemapInputNames {
		inNameMap.Store(newName, oldName)
	}
	outNameMap := &sync.Map{}
	for oldName, newName := range params.RemapOutputNames {
		outNameMap.Store(newName, oldName)
	}
	if len(params.BoxOrder) != 0 {
		if len(params.BoxOrder) != 4 {
			return nil, errors.Errorf(
				"attribute xmin_ymin_xmax_ymax_order for model %q must have only 4 entries in the list. Got %v",
				params.ModelName,
				params.BoxOrder,
			)
		}
		checkOrder := map[int]bool{0: false, 1: false, 2: false, 3: false}
		for _, entry := range params.BoxOrder {
			val, ok := checkOrder[entry]
			if !ok || val { // if val is true, it means value was repeated
				return nil, errors.Errorf(
					"attribute xmin_ymin_xmax_ymax_order for model %q can only have entries 0, 1, 2 and 3, and only one instance of each. Got %v",
					params.ModelName,
					params.BoxOrder,
				)
			}
			checkOrder[entry] = true
		}
	}
	var errList []error
	classifierFunc, err := attemptToBuildClassifier(mlm, inNameMap, outNameMap, params)
	if err != nil {
		errList = append(errList, err)
		logger.CDebugw(ctx, "unable to use ml model as a classifier, will attempt to evaluate as"+
			"detector and segmenter", "model", params.ModelName, "error", err)
	} else {
		err := checkIfClassifierWorks(ctx, classifierFunc)
		errList = append(errList, err)
		if err != nil {
			classifierFunc = nil
			logger.CDebugw(ctx, "unable to use ml model as a classifier, will attempt to evaluate as detector"+
				" and 3D segmenter", "model", params.ModelName, "error", err)
		} else {
			logger.CInfow(ctx, "model fulfills a vision service classifier", "model", params.ModelName)
		}
	}

	detectorFunc, err := attemptToBuildDetector(mlm, inNameMap, outNameMap, params)
	if err != nil {
		errList = append(errList, err)
		logger.CDebugw(ctx, "unable to use ml model as a detector, will attempt to evaluate as 3D segmenter",
			"model", params.ModelName, "error", err)
	} else {
		err = checkIfDetectorWorks(ctx, detectorFunc)
		errList = append(errList, err)
		if err != nil {
			detectorFunc = nil
			logger.CDebugw(ctx, "unable to use ml model as a detector, will attempt to evaluate as 3D segmenter",
				"model", params.ModelName, "error", err)
		} else {
			logger.CInfow(ctx, "model fulfills a vision service detector", "model", params.ModelName)
		}
	}

	segmenter3DFunc, err := attemptToBuild3DSegmenter(mlm, inNameMap, outNameMap)
	errList = append(errList, err)
	if err != nil {
		logger.CDebugw(ctx, "unable to use ml model as 3D segmenter", "model", params.ModelName, "error", err)
	} else {
		logger.CInfow(ctx, "model fulfills a vision service 3D segmenter", "model", params.ModelName)
	}

	// If nothing worked, give more info
	if errList[0] != nil && errList[1] != nil && errList[2] != nil {
		for _, e := range errList {
			logger.Error(e)
		}
		md, err := mlm.Metadata(ctx)
		if err != nil {
			logger.Error("could not get metadata from the model")
		} else {
			inputs := ""
			for _, tensor := range md.Inputs {
				inputs += fmt.Sprintf("%s(%v) ", tensor.Name, tensor.Shape)
			}
			outputs := ""
			for _, tensor := range md.Outputs {
				outputs += fmt.Sprintf("%s(%v) ", tensor.Name, tensor.Shape)
			}
			logger.Infow("the model has the following input and outputs tensors, name(shape)",
				"inputs", inputs,
				"outputs", outputs,
			)
		}
	}

	if params.DefaultCamera != "" {
		_, err = camera.FromRobot(r, params.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", params.DefaultCamera)
		}
	}

	// Don't return a close function, because you don't want to close the underlying ML service
	return vision.DeprecatedNewService(name, r, nil, classifierFunc, detectorFunc, segmenter3DFunc, params.DefaultCamera)
}

func getLabelsFromFile(labelPath string) []string {
	var labels []string
	f, err := os.Open(filepath.Clean(labelPath))
	if err != nil {
		return nil
	}
	defer func() {
		if err := f.Close(); err != nil {
			logger := logging.NewLogger("labelFile")
			logger.Warnw("could not get labels from file", "error", err)
			return
		}
	}()
	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		label := strings.TrimSpace(scanner.Text())
		if label == "" {
			continue
		}
		labels = append(labels, scanner.Text())
	}
	// if the labels come out as one line, try splitting that line by spaces or commas to extract labels
	// Check if the labels should be comma split first and then space split.
	if len(labels) == 1 {
		labels = strings.Split(labels[0], ",")
	}
	if len(labels) == 1 {
		labels = strings.Split(labels[0], " ")
	}
	return labels
}

// getLabelsFromMetadata returns a slice of strings--the intended labels.
func getLabelsFromMetadata(md mlmodel.MLMetadata, labelPath string) []string {
	if labelPath != "" {
		return getLabelsFromFile(labelPath)
	}
	if len(md.Outputs) < 1 {
		return nil
	}

	if labelPath, ok := md.Outputs[0].Extra["labels"].(string); ok {
		if labelPath == "" { // no label file specified
			return nil
		}
		labels := getLabelsFromFile(labelPath)
		if len(labels) != 0 {
			return labels
		}
	}
	return nil
}

// getBoxOrderFromMetadata returns a slice of ints--the bounding box
// display order, where 0=xmin, 1=ymin, 2=xmax, 3=ymax.
func getBoxOrderFromMetadata(md mlmodel.MLMetadata) ([]int, error) {
	for _, o := range md.Outputs {
		if strings.Contains(o.Name, "location") {
			out := make([]int, 0, 4)
			if order, ok := o.Extra["boxOrder"].([]uint32); ok {
				for _, o := range order {
					out = append(out, int(o))
				}
				return out, nil
			}
		}
	}
	return nil, errors.New("could not grab bbox order")
}
