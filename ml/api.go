// Package ml provides some fundamental machine learning primitives.
package ml

import (
	"gorgonia.org/tensor"
)

// Tensors are a data structure to hold the input and output map of tensors that will fed into a
// model, or come from the result of a model.
type Tensors map[string]*tensor.Dense
