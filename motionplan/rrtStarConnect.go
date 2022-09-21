package motionplan

import (
	"context"
	"encoding/json"
	"math"
	"math/rand"

	"github.com/edaniels/golog"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/utils"
	"gonum.org/v1/gonum/mat"

	"go.viam.com/rdk/referenceframe"
)

const (
	// If a solution is found that is within this percentage of the optimal unconstrained solution, exit early.
	defaultOptimalityThreshold = .05

	// Period of iterations after which a new solution is calculated and updated.
	defaultSolutionCalculationPeriod = 100

	// The number of nearest neighbors to consider when adding a new sample to the tree.
	defaultNeighborhoodSize = 10
)

type rrtStarConnectOptions struct {
	// If a solution is found that is within this percentage of the optimal unconstrained solution, exit early
	OptimalityThreshold float64 `json:"optimality_threshold"`

	// Period of iterations after which a new solution is calculated and updated.
	SolutionCalculationPeriod int

	// The number of nearest neighbors to consider when adding a new sample to the tree
	NeighborhoodSize int `json:"neighborhood_size"`

	// Parameters common to all RRT implementations
	*rrtOptions
}

// newRRTStarConnectOptions creates a struct controlling the running of a single invocation of the algorithm.
// All values are pre-set to reasonable defaults, but can be tweaked if needed.
func newRRTStarConnectOptions(planOpts *PlannerOptions) (*rrtStarConnectOptions, error) {
	algOpts := &rrtStarConnectOptions{
		OptimalityThreshold:       defaultOptimalityThreshold,
		SolutionCalculationPeriod: defaultSolutionCalculationPeriod,
		NeighborhoodSize:          defaultNeighborhoodSize,
		rrtOptions:                newRRTOptions(planOpts),
	}
	// convert map to json
	jsonString, err := json.Marshal(planOpts.extra)
	if err != nil {
		return nil, err
	}
	err = json.Unmarshal(jsonString, algOpts)
	if err != nil {
		return nil, err
	}
	return algOpts, nil
}

// rrtStarConnectMotionPlanner is an object able to asymptotically optimally path around obstacles to some goal for a given referenceframe.
// It uses the RRT*-Connect algorithm, Klemm et al 2015
// https://ieeexplore.ieee.org/document/7419012
type rrtStarConnectMotionPlanner struct{ *planner }

// NewRRTStarConnectMotionPlanner creates a rrtStarConnectMotionPlanner object.
func NewRRTStarConnectMotionPlanner(frame referenceframe.Frame, nCPU int, logger golog.Logger) (MotionPlanner, error) {
	//nolint:gosec
	return NewRRTStarConnectMotionPlannerWithSeed(frame, nCPU, rand.New(rand.NewSource(1)), logger)
}

// NewRRTStarConnectMotionPlannerWithSeed creates a rrtStarConnectMotionPlanner object with a user specified random seed.
func NewRRTStarConnectMotionPlannerWithSeed(
	frame referenceframe.Frame,
	nCPU int,
	seed *rand.Rand,
	logger golog.Logger,
) (MotionPlanner, error) {
	planner, err := newPlanner(frame, nCPU, seed, logger)
	if err != nil {
		return nil, err
	}
	return &rrtStarConnectMotionPlanner{planner}, nil
}

func (mp *rrtStarConnectMotionPlanner) Plan(ctx context.Context,
	goal *commonpb.Pose,
	seed []referenceframe.Input,
	planOpts *PlannerOptions,
) ([][]referenceframe.Input, error) {
	if planOpts == nil {
		planOpts = NewBasicPlannerOptions()
	}
	solutionChan := make(chan *planReturn, 1)
	utils.PanicCapturingGo(func() {
		mp.planRunner(ctx, goal, seed, planOpts, nil, solutionChan)
	})
	select {
	case <-ctx.Done():
		return nil, ctx.Err()
	case plan := <-solutionChan:
		return plan.toInputs(), plan.err
	}
}

// planRunner will execute the plan. When Plan() is called, it will call planRunner in a separate thread and wait for the results.
// Separating this allows other things to call planRunner in parallel while also enabling the thread-agnostic Plan to be accessible.
func (mp *rrtStarConnectMotionPlanner) planRunner(ctx context.Context,
	goal *commonpb.Pose,
	seed []referenceframe.Input,
	planOpts *PlannerOptions,
	endpointPreview chan *node,
	solutionChan chan *planReturn,
) {
	defer close(solutionChan)

	// setup planner options
	if planOpts == nil {
		planOpts = NewBasicPlannerOptions()
	}
	algOpts, err := newRRTStarConnectOptions(planOpts)
	if err != nil {
		solutionChan <- &planReturn{err: err}
		return
	}

	// get many potential end goals from IK solver
	solutions, err := getSolutions(ctx, planOpts, mp.solver, goal, seed, mp.Frame())
	if err != nil {
		solutionChan <- &planReturn{err: err}
		return
	}

	// publish endpoint of plan if it is known
	if planOpts.MaxSolutions == 1 && endpointPreview != nil {
		endpointPreview <- solutions[0]
	}

	// the smallest interpolated distance between the start and end input represents a lower bound on cost
	optimalCost := solutions[0].cost

	// initialize maps
	goalMap := make(map[*node]*node, len(solutions))
	for _, solution := range solutions {
		goalMap[&node{q: solution.q, cost: 0}] = nil
	}
	startMap := make(map[*node]*node)
	startMap[&node{q: seed}] = nil

	// for the first iteration, we try the 0.5 interpolation between seed and goal[0]
	samples := [][]referenceframe.Input{referenceframe.InterpolateInputs(seed, solutions[0].q, 0.5)}

	// Create a reference to the two maps so that we can alternate which one is grown
	map1, map2 := startMap, goalMap

	// Keep a list of the node pairs that have the same inputs
	shared := make([]*nodePair, 0)

	// sample until the max number of iterations is reached
	var solutionCost float64
	for i := 0; i < algOpts.PlanIter; i++ {
		select {
		case <-ctx.Done():
			solutionChan <- &planReturn{err: ctx.Err()}
			return
		default:
		}

		// try to connect the target to map 1
		target := samples[i%algOpts.SolutionCalculationPeriod]
		if map1reached := mp.extend(algOpts, map1, target); map1reached != nil {
			// try to connect the target to map 2
			if map2reached := mp.extend(algOpts, map2, target); map2reached != nil {
				// target was added to both map
				shared = append(shared, &nodePair{map1reached, map2reached})
			}
		}

		// calculate the solution and log status of planner
		if i%algOpts.SolutionCalculationPeriod == 0 {
			solution := shortestPath(startMap, goalMap, shared)
			solutionCost = EvaluatePlan(solution, planOpts)
			mp.logger.Debugf("RRT* progress: %d%%\tpath cost: %.3f", 100*i/algOpts.PlanIter, solutionCost)

			// check if an early exit is possible
			if solutionCost-optimalCost < algOpts.OptimalityThreshold*optimalCost {
				mp.logger.Debug("RRT* progress: sufficiently optimal path found, exiting")
				solutionChan <- solution
				return
			}
			samples = mp.sample(algOpts, solutionCost, seed, solutions[len(solutions)-1].q)
		}

		// get next sample, switch map pointers
		map1, map2 = map2, map1
	}

	solutionChan <- shortestPath(startMap, goalMap, shared)
}

func (mp *rrtStarConnectMotionPlanner) extend(algOpts *rrtStarConnectOptions, tree map[*node]*node, target []referenceframe.Input) *node {
	if validTarget := mp.checkInputs(algOpts.planOpts, target); !validTarget {
		return nil
	}

	// iterate over the k nearest neighbors and find the minimum cost to connect the target node to the tree
	neighbors := kNearestNeighbors(algOpts.planOpts, tree, target, algOpts.NeighborhoodSize)
	minCost := math.Inf(1)
	var minIndex int
	for i, neighbor := range neighbors {
		cost := neighbor.node.cost + neighbor.dist
		if cost < minCost && mp.checkPath(algOpts.planOpts, neighbor.node.q, target) {
			minIndex = i
			minCost = cost
		}
	}

	// add new node to tree as a child of the minimum cost neighbor node
	targetNode := &node{q: target, cost: minCost}
	tree[targetNode] = neighbors[minIndex].node

	// rewire the tree
	for i, neighbor := range neighbors {
		// dont need to try to rewire minIndex, so skip it
		if i == minIndex {
			continue
		}

		// check to see if a shortcut is possible, and rewire the node if it is
		_, connectionCost := algOpts.planOpts.DistanceFunc(&ConstraintInput{
			StartInput: neighbor.node.q,
			EndInput:   targetNode.q,
		})
		cost := connectionCost + targetNode.cost
		if cost < neighbor.node.cost && mp.checkPath(algOpts.planOpts, target, neighbor.node.q) {
			neighbor.node.cost = cost
			tree[neighbor.node] = targetNode
		}
	}
	return targetNode
}

func (mp *rrtStarConnectMotionPlanner) sample(
	algOpts *rrtStarConnectOptions,
	bestCost float64,
	start, goal []referenceframe.Input,
) [][]referenceframe.Input {
	if bestCost < math.Inf(1) {
		return mp.informed_sample(
			algOpts,
			bestCost,
			mat.NewVecDense(len(start), referenceframe.InputsToFloats(start)),
			mat.NewVecDense(len(goal), referenceframe.InputsToFloats(goal)),
		)
	}
	samples := make([][]referenceframe.Input, 0)
	for i := 0; i < algOpts.SolutionCalculationPeriod; i++ {
		samples = append(samples, referenceframe.RandomFrameInputs(mp.frame, mp.randseed))
	}
	return samples
}

func (mp *rrtStarConnectMotionPlanner) informed_sample(
	algOpts *rrtStarConnectOptions,
	bestCost float64,
	start, goal *mat.VecDense,
) [][]referenceframe.Input {
	n := start.Len()

	// compute center of ellipse
	center := &mat.VecDense{}
	center.AddScaledVec(start, .5, goal)

	// compute cmin
	difference := &mat.VecDense{}
	difference.SubVec(goal, start)
	minCost := difference.Norm(2)

	// compute ellipse geometry
	r1 := bestCost / 2
	r2 := math.Sqrt(bestCost*bestCost - minCost*minCost)

	// construct M matrix
	a := &mat.VecDense{}
	a.ScaleVec(1/minCost, difference)
	M := mat.NewDense(n, n, nil)
	M.SetCol(0, a.RawVector().Data)

	// compute SVD of M
	U, V := &mat.Dense{}, &mat.Dense{}
	var svd mat.SVD
	ok := svd.Factorize(M, mat.SVDFull)
	if !ok {
		return nil
	}
	svd.UTo(U)
	svd.VTo(V)

	// compute L matrix
	lDiag := make([]float64, n)
	lDiag[0] = r1
	for i := 1; i < n; i++ {
		lDiag[i] = r2 / 2
	}
	L := mat.NewDiagDense(n, lDiag)

	// compute C matrix
	sigDiag := make([]float64, n)
	for i := 0; i < n-1; i++ {
		sigDiag[i] = 1
	}
	sigDiag[n-1] = mat.Det(U) * mat.Det(U)
	Sigma := mat.NewDiagDense(n, sigDiag)
	tempResult, C := &mat.Dense{}, &mat.Dense{}
	tempResult.Mul(U, Sigma)
	C.Mul(tempResult, V.T())

	// compute final samples
	tempResult.Mul(C, L)
	samples := make([][]referenceframe.Input, 0)
	for i := 0; i < algOpts.SolutionCalculationPeriod; i++ {
		scaledSample, sampleVec := &mat.VecDense{}, &mat.VecDense{}
		scaledSample.MulVec(tempResult, mp.sampleBall(n))
		sampleVec.AddVec(scaledSample, center)

		// make sure samples fall within bounds of frame
		limits := mp.frame.DoF()
		sample := sampleVec.RawVector().Data
		for i := 0; i < n; i++ {
			if sample[i] < limits[i].Min {
				sample[i] = limits[i].Min
			} else if sample[i] > limits[i].Max {
				sample[i] = limits[i].Max
			}
		}
		samples = append(samples, referenceframe.FloatsToInputs(sample))
	}

	return samples
}

// source: http://extremelearning.com.au/how-to-generate-uniformly-random-points-on-n-spheres-and-n-balls/
func (mp *rrtStarConnectMotionPlanner) sampleBall(n int) *mat.VecDense {
	rands := make([]float64, n)
	for j := 0; j < n; j++ {
		rands[j] = mp.randseed.NormFloat64()
	}
	r := math.Pow(mp.randseed.Float64(), 1/float64(n))
	u := mat.NewVecDense(n, rands)
	var sample *mat.VecDense
	sample.ScaleVec(r/u.Norm(2), u)
	return sample
}
