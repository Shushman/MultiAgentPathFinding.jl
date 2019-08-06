# Strongly influenced by
module MultiAgentPathFinding

# stdlib
using Random
using DataStructures

# others
using Parameters
using Graphs

# Imports
import Base: isequal, isless, isempty
import DataStructures: compare



# Abstract types
abstract type MAPFState end
abstract type MAPFAction end
abstract type MAPFConflict end
abstract type MAPFConstraints end
abstract type MAPFEnvironment end

export
    MAPFState,
    MAPFAction,
    MAPFConflict,
    MAPFConstraints,
    MAPFEnvironment


export
    PlanResult,
    get_mapf_state_from_idx,
    get_mapf_action,
    get_plan_result_from_astar

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
include("cbs.jl")
include("ecbs.jl")
include("domains/grid2d/types.jl")
include("domains/grid2d/cbs_grid2d.jl")
include("domains/grid2d/ecbs_grid2d.jl")
include("domains/grid2d/a_star_epsilon_grid2d.jl")

end # module
