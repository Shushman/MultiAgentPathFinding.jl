## Some constraint functions can carry over (but not all)

# Breaks the agnostic-ness to the number of agents but that's fine
@with_kw mutable struct Grid2DCGEnvironment <: MAPFEnvironment
    grid2d_env::Grid2DEnvironment
    agent_ground::Vector{Bool}
end

function get_mapf_state_from_idx(env::Grid2DCGEnvironment, idx::Int64)
    grid2d_state = get_mapf_state_from_idx(env.grid2d_env, idx)
    return Grid2DCGState(grid2d_state, 0)
end

function get_mapf_action(env::Grid2DCGEnvironment, source::Int64, target::Int64)
    grid2d_action = get_mapf_action(env.grid2d_env, source, target)
    return Grid2DCGAction(grid2d_action.action, 0)   # Default is no coordination
end

set_low_level_context!(env::Grid2DCGEnvironment, agent_idx::Int64, constraints::Grid2DConstraints) = set_low_level_context!(env.grid2d_env, agent_idx, constraints)

# Heuristic and is_solution are for a_star

function get_phase1_soln_constraints(env::Grid2DCGEnvironment, initial_states::Vector{Grid2DCGState})

    @assert length(env.agent_ground) == length(initial_states)

    # Create solver for underlying environment
    phase1_cbs_solver = CBSSolver{Grid2DState,Grid2DAction,Int64,SumOfCosts,Grid2DConflict,Grid2DConstraints,Grid2DEnvironment}(env=env.grid2d_env)

    init_ground_states = Grid2DState[]
    init_air_states = Grid2DState[]

    for (i,s) in enumerate(initial_states)
        if env.agent_ground[i] == true
            push!(init_ground_states, s)
        else
            push!(init_air_states, s)
        end
    end # enumerate(initial_states)

    ground_soln, ground_constr = search!(phase1_cbs_solver, init_ground_states)
    air_soln, air_constr = search!(phase1_cbs_solver, init_air_states)

    phase1_soln = cat(ground_soln, air_soln, dims=1)
    phase1_constraints = cat(ground_constr, air_constr, dims=1)

    return phase1_soln, phase1_constraints
end

# Now need to implement the four functions!
function get_coord_graph_from_states(env::Grid2DCGEnvironment, curr_states::Vector{Grid2DCGState})

    num_agents = length(curr_states)
    cg_adj_mat = zeros(Int64, num_agents, num_agents)

    for (i, si) in enumerate(curr_states)
        for (j, sj) in enumerate(curr_states)

            # Will have no nodes of degree > 1
            if env.agent_ground[i] != env.agent_ground[j] &&
                equal_except_time(s1.grid2d_state, s2.grid2d_state)
                cg_adj_mat[i ,j] = 1
                cg_adj_mat[j, i] = 1
            end

        end
    end
    
    return SimpleGraph(cg_adj_mat)
end


function get_all_mapf_actions_between_states(env::Grid2DCGEnvironment, s1::Grid2DCGState, s2::Grid2DCGState)

    # Only two coordination modes: 0 for nothing and 1 for coord
    state1 = s1.grid2d_state
    state2 = s2.grid2d_state

    if isequal(state1, state2)
        return []
    end

    if equal_except_time(state1, state2)
        return [Grid2DCGAction(action=Wait, coord_mode=0)] # Can't wait and coordinate
    else
        if state2.x > state1.x
            return [Grid2DCGAction(action=Right, coord_mode=0), Grid2DCGAction(action=Right, coord_mode=1)]
        elseif state2.x < state1.x
            return [Grid2DCGAction(action=Left, coord_mode=0), Grid2DCGAction(action=Left, coord_mode=1)]
        elseif state2.y > state1.y
            return [Grid2DCGAction(action=Up, coord_mode=0), Grid2DCGAction(action=Up, coord_mode=1)]
        else
            return [Grid2DCGAction(action=Down, coord_mode=0), Grid2DCGAction(action=Down, coord_mode=1)]
        end
    end
end


function indiv_agent_cost(env::Grid2DCGEnvironment, idx::Int64, s::Grid2DCGState, a::Grid2DCGAction)

    # TODO
    if env.agent_ground[idx] == true
        return 1
    else
        # air - cost is 0 if coordinating and so is action
        if s.coord_mode == 1 && a.coord_mode == 1
            return 0
        else
            return 1
        end
    end

end


function get_coord_cost(env::Grid2DCGEnvironment, idx1::Int64, idx2::Int64, s1::Grid2DCGState, s2::Grid2DCGState, a1::Grid2DCGAction, a2::Grid2DCGAction)

    # Should never have opposite coord modes
    if a1.coord_mode != a2.coord_mode
        return typemax(Int64)
    end

    if s1.coord_mode == 0 && s2.coord_mode == 0
        if a1.coord_mode == 1 && a2.coord_mode == 1
                if a1.action == a2.action
                    # When doing same thing, doing together has a bonus
                    return -1
                else
                    # When doing different things, should not coordinate
                    return typemax(Int64)
                end
        else # Neither coordinates
            return 0
        end
    elseif s1.coord_mode == 1 && s2.coord_mode == 1
        # Currently coordinating; coordinate unless next action different
        if a1.action != a2.action && a1.coord_mode == 1
            return typemax(Int64) # Don't coordinate when next action differs
        end
    end

    return 0
end

function update_states_with_coord_action(state_seq::Vector{Grid2DCGState}, action::Grid2DCGAction, curr_time::Int64)

    next_state = state_seq[curr_time+1]
    if action.coord_mode == 1
        next_state.coord_mode = 1
    else
        next_state.coord_mode = 0
    end

    return next_state
end