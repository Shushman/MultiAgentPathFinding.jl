"""
    PlanResult{S <: MAPFState, A <: MAPFAction, C <: Number}

Stores the result of an individual agent's search.

Attributes:
    - `states::Vector{Tuple{S,C}}` The list of (state, cost-to-go) on the path
    - `actions::Vector{Tuple{A,C}}` The list of (actions, cost) on the path.
    - `cost::C` The cumulative cost of the path
    - `fmin::C` The minimum f-value expanded during search
"""
@with_kw struct PlanResult{S <: MAPFState, A <: MAPFAction, C <: Number}
    states::Vector{Tuple{S,C}}  = Vector{Tuple{S,C}}(undef, 0)
    actions::Vector{Tuple{A,C}} = Vector{Tuple{A,C}}(undef, 0)
    cost::C                     = zero(C)
    fmin::C                     = zero(C)
end

Base.isempty(pln::PlanResult) = isempty(pln.states)

"""
    get_mapf_state_from_idx(env::E, idx::Int64) where {E <: MAPFEnvironment}

My A* search implementation returns a list of graph indices; this function maps an index
to the corresponding state for that environment.
"""
function get_mapf_state_from_idx end

"""
    get_mapf_action(env::E, source::Int64, target::Int64) where {E <: MAPFEnvironment}

Returns the MAPFAction while going from source index to target index.
"""
function get_mapf_action end

"""
    get_plan_result_from_astar(env::E, a_star_dists::Dict, a_star_parent_indices::Dict,
                               start_idx::Int64, goal_idx::Int64,
                               best_fvalue::D) where {E <: MAPFEnvironment, D <: Number}

Converts the result of an A*-search (from my Graphs.jl implementation) to a corresponding PlanResult instance.
"""
function get_plan_result_from_astar(env::E, a_star_dists::Dict, a_star_parent_indices::Dict,
                                    start_idx::Int64, goal_idx::Int64,
                                    best_fvalue::D) where {E <: MAPFEnvironment, D <: Number}

    # First ensure the goal is reachable
    if ~(haskey(a_star_dists, goal_idx))
        return nothing
    end

    # Set both cost and fmin
    cost = a_star_dists[goal_idx]
    if best_fvalue == zero(D)
        fmin = a_star_dists[goal_idx]
    else
        fmin = best_fvalue
    end

    # Insert last elements to states and actions arrays
    goal_state = get_mapf_state_from_idx(env, goal_idx)
    states = [(goal_state, cost)]

    curr_idx = a_star_parent_indices[goal_idx]

    goal_action = get_mapf_action(env, curr_idx, goal_idx)
    action_cost = a_star_dists[goal_idx] - a_star_dists[curr_idx]
    actions = [(goal_action, action_cost)]

    # Now walk back to start
    while curr_idx != start_idx
        prev_idx = a_star_parent_indices[curr_idx]

        # Update states
        pushfirst!(states, (get_mapf_state_from_idx(env, curr_idx), a_star_dists[curr_idx]))

        # Update actions
        pushfirst!(actions, (get_mapf_action(env, prev_idx, curr_idx),
                        a_star_dists[curr_idx] - a_star_dists[prev_idx]))

        curr_idx = prev_idx
    end

    pushfirst!(states, (get_mapf_state_from_idx(env, start_idx), 0))

    return PlanResult(states=states, actions=actions, cost=cost, fmin=fmin)
end
