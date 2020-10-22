module MultiAgentPathFinding

# stdlib
using Random
using DataStructures

# others
using Parameters
using Graphs
using LightGraphs

# Imports
import Base: isequal, isless, isempty
import DataStructures: compare



# Abstract types
""" MAPFState

Abstract base type for tracking the state of an agent in a MAPF problem
"""
abstract type MAPFState end

""" MAPFAction

Abstract base type for the action that an agent can take in a MAPF problem.
"""
abstract type MAPFAction end

""" MAPFConflict

Abstract base type for a path-path conflict in a MAPF problem.
"""
abstract type MAPFConflict end

""" MAPFConstraints

Abstract base type for the constraints on the low-level path search induced by a conflict in a MAPF problem.
"""
abstract type MAPFConstraints end

""" MAPFEnvironment

Abstract base type for the environment in a MAPF problem.
"""
abstract type MAPFEnvironment end

# Types
export
    MAPFState,
    MAPFAction,
    MAPFConflict,
    MAPFConstraints,
    MAPFEnvironment

# Utils
export
    PlanResult,
    get_mapf_state_from_idx,
    get_mapf_action,
    get_plan_result_from_astar

# High Level Cost elements
export
    HighLevelCost,
    compute_cost,
    accumulate_cost,
    deaccumulate_cost,
    SumOfCosts,
    Makespan,
    SumOfCoordinatedCosts

# CBS elements
export
    get_empty_constraint,
    set_low_level_context!,
    get_first_conflict,
    create_constraints_from_conflict,
    overlap_between_constraints,
    add_constraint,
    low_level_search!,
    CBSSolver,
    search!,
    focal_heuristic,
    ECBSSolver


# CBS-CG specific elements
export
    get_coord_graph_from_states,
    get_coord_cost,
    get_all_mapf_actions_between_states,
    update_states_with_coord_action,
    get_phase1_soln_constraints,
    indiv_agent_cost,
    CBSCGHighLevelNode,
    CBSCGSolver

# Grid 2D Types
export
    Grid2DState,
    Grid2DAction,
    Grid2DConflict,
    Grid2DConstraints,
    Grid2DEnvironment,
    Grid2DLocation,
    AStarGrid2DEnvironment

include("utils.jl")
include("high_level_cost.jl")
include("cbs.jl")
include("ecbs.jl")
include("cbs_coord_graphs.jl")
include("domains/grid2d/types.jl")
include("domains/grid2d/cbs_grid2d.jl")
include("domains/grid2d/ecbs_grid2d.jl")
include("domains/grid2d/a_star_epsilon_grid2d.jl")
include("domains/grid2d_cg/grid2d_cg_types.jl")
include("domains/grid2d_cg/cbs_grid2d_cg.jl")

end # module
