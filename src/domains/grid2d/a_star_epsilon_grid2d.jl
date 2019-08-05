@with_kw mutable struct AStarGrid2DEnvironment <: MAPFEnvironment
    dimx::Int64
    dimy::Int64
    obstacles::Set{Grid2DLocation}
    goal::Grid2DLocation
    state_graph::SimpleVListGraph{Grid2DLocation}    = SimpleVListGraph{Grid2DLocation}()
    state_to_idx::Dict{Grid2DLocation,Int64}         = Dict{Grid2DLocation,Int64}()
    goal_idx::Int64                                  = 0
end

function admissible_heuristic_a_star_eps(env::AStarGrid2DEnvironment, s::Grid2DLocation)
    return abs(s.x - env.goal.x) + abs(s.y - env.goal.y)
end

function focal_state_heuristic_a_star_eps(env::AStarGrid2DEnvironment, s::Grid2DLocation, gscore::Int64)
    return gscore
end

function focal_transition_heuristic_a_star_eps(env::AStarGrid2DEnvironment, s1::Grid2DLocation, s2::Grid2DLocation,
                                    gscore1::Int64, gscore2::Int64)
    return gscore2 - gscore1
end

function state_valid(env::AStarGrid2DEnvironment, s::Grid2DLocation)
    return s.x >=0 && s.x < env.dimx && s.y >=0 && s.y < env.dimy &&
            ~(s in env.obstacles)
end

function get_env_state_idx!(env::AStarGrid2DEnvironment, s::Grid2DLocation)

    idx = get(env.state_to_idx, s, 0)

    if idx == 0
        add_vertex!(env.state_graph, s)
        nbr_idx = num_vertices(env.state_graph)
        env.state_to_idx[s] = nbr_idx
        idx = nbr_idx
    end

    return idx
end

function get_mapf_state_from_idx(env::AStarGrid2DEnvironment, idx::Int64)
    return env.state_graph.vertices[idx]
end

function get_mapf_action(env::AStarGrid2DEnvironment, source::Int64, target::Int64)

    state1 = env.state_graph.vertices[source]
    state2 = env.state_graph.vertices[target]

    if state2.x > state1.x
        return Grid2DAction(Right::Action)
    elseif state2.x < state1.x
        return Grid2DAction(Left::Action)
    elseif state2.y > state1.y
        return Grid2DAction(Up::Action)
    else
        return Grid2DAction(Down::Action)
    end
    return nothing
end

function low_level_search!(env::AStarGrid2DEnvironment, s::Grid2DLocation, weight::Float64)

    source_idx = get_env_state_idx!(env, s)

    # Set all edge weights to one
    edge_wt_fn(u, v) = 1

    # Set the heuristics
    admissible_heuristic(s) = admissible_heuristic_a_star_eps(env, s)
    focal_state_heuristic(s, gs) = focal_state_heuristic_a_star_eps(env, s, gs)
    focal_transition_heuristic(s1, s2, gs1, gs2) = focal_transition_heuristic_a_star_eps(env, s1, s2, gs1, gs2)

    # Run the search
    a_star_epsilon_states = a_star_light_epsilon_shortest_path_implicit!(env.state_graph,
                                                                        edge_wt_fn, source_idx,
                                                                        AStarEpsilonGrid2DVisitor(env),
                                                                        weight,
                                                                        admissible_heuristic,
                                                                        focal_state_heuristic,
                                                                        focal_transition_heuristic,
                                                                        Int64)


    # Extract the solution as a vector of locations and dist tuples
    if ~(haskey(a_star_epsilon_states.dists, env.goal_idx))
        return nothing, 0
    end

    cost = a_star_epsilon_states.dists[env.goal_idx]

    goal_state = env.state_graph.vertices[env.goal_idx]
    solution = [goal_state]

    curr_idx = env.goal_idx
    while curr_idx != source_idx

        prev_idx = a_star_epsilon_states.parent_indices[curr_idx]
        pushfirst!(solution, env.state_graph.vertices[prev_idx])
        curr_idx = prev_idx

    end

    return solution, cost
end



mutable struct AStarEpsilonGrid2DVisitor <: AbstractDijkstraVisitor
    env::AStarGrid2DEnvironment
end

function Graphs.include_vertex!(vis::AStarEpsilonGrid2DVisitor, u::Grid2DLocation, v::Grid2DLocation,
                                d::N, nbrs::Vector{Int64}) where {N <: Number}

    env = vis.env
    if v == env.goal
        env.goal_idx = get_env_state_idx!(env, v)
        return false
    end

    empty!(nbrs)

    temp_locs = [Grid2DLocation((x = v.x-1, y = v.y)), Grid2DLocation((x = v.x+1, y = v.y)),
                 Grid2DLocation((x = v.x, y = v.y-1)), Grid2DLocation((x = v.x, y = v.y+1))]

    for temp_loc in temp_locs
        if state_valid(env, temp_loc)
            nbr_idx = get_env_state_idx!(env, temp_loc)
            push!(nbrs, nbr_idx)
        end
    end

    return true
end
