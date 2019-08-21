# Store the result of a single agent plan
@with_kw struct PlanResult{S <: MAPFState, A <: MAPFAction, C <: Number}
    states::Vector{Tuple{S,C}}  = Vector{Tuple{S,C}}(undef, 0)
    actions::Vector{Tuple{A,C}} = Vector{Tuple{A,C}}(undef, 0)
    cost::C                     = zero(C)
    fmin::C                     = zero(C)
end

Base.isempty(pln::PlanResult) = isempty(pln.states)


function get_mapf_state_from_idx end
function get_mapf_action end

# Given a MAPF Environment, state-action types and A-star parent index
# generate the PlanResult
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
