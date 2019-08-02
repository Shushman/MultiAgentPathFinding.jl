@with_kw struct Grid2DState <: MAPFState
    time::Int64
    x::Int64
    y::Int64
end

Base.isequal(s1::Grid2DState, s2::Grid2DState) = (s1.time, s1.x, s1.y) == (s2.time, s2.x, s2.y)

function equal_except_time(s1::Grid2DState, s2::Grid2DState)
    return (s1.x == s2.x) && (s1.y == s2.y)
end

@enum Action Up=1 Down=2 Left=3 Right=4 Wait=5

struct Grid2DAction <: MAPFAction
    action::Action
end

@enum ConflictType Vertex=1 Edge=2

@with_kw struct Grid2DConflict <: MAPFConflict
    time::Int64
    agent_1::Int64
    agent_2::Int64
    type::ConflictType
    x1::Int64       = -1
    y1::Int64       = -1
    x2::Int64       = -1
    y2::Int64       = -1
end


@with_kw struct VertexConstraint
    time::Int64
    x::Int64
    y::Int64
end

Base.isless(vc1::VertexConstraint, vc2::VertexConstraint) = (vc1.time, vc1.x, vc1.x) < (vc2.time, vc2.x, vc2.y)
Base.isequal(vc1::VertexConstraint, vc2::VertexConstraint) = (vc1.time, vc1.x, vc1.y) == (vc2.time, vc2.x, vc2.y)

@with_kw struct EdgeConstraint
    time::Int64
    x1::Int64
    y1::Int64
    x2::Int64
    y2::Int64
end

Base.isless(ec1::EdgeConstraint, ec2::EdgeConstraint) = (ec1.time, ec1.x1, ec1.y1, ec1.x2, ec1.y2) < (ec2.time, ec2.x1, ec2.y1, ec2.x2, ec2.y2)
Base.isequal(ec1::EdgeConstraint, ec2::EdgeConstraint) = (ec1.time, ec1.x1, ec1.y1, ec1.x2, ec1.y2) == (ec2.time, ec2.x1, ec2.y1, ec2.x2, ec2.y2)

@with_kw struct Grid2DConstraints <: MAPFConstraints
    vertex_constraints::Set{VertexConstraint}   = Set{VertexConstraint}()
    edge_constraints::Set{EdgeConstraint}       = Set{EdgeConstraint}()
end

get_empty_constraint(Grid2DConstraints) = Grid2DConstraints()

Base.isempty(constraints::Grid2DConstraints) = (isempty(constraints.vertex_constraints) && isempty(constraints.edge_constraints))

# Add cadd to cbase
function add_constraint!(cbase::Grid2DConstraints, cadd::Grid2DConstraints)

    union!(cbase.vertex_constraints, cadd.vertex_constraints)
    union!(cbase.edge_constraints, cadd.edge_constraints)

end

function overlap_between_constraints(cbase::Grid2DConstraints, cother::Grid2DConstraints)

    vertex_intersect = intersect(cbase.vertex_constraints, cother.vertex_constraints)
    edge_intersect = intersect(cbase.edge_constraints, cother.edge_constraints)

    return ~(isempty(vertex_intersect)) || ~(isempty(edge_intersect))

end

const Grid2DLocation = NamedTuple{(:x,:y),Tuple{Int64,Int64}}

# Not setting agent_idx and constraints as they will always be there for functions
@with_kw mutable struct Grid2DEnvironment <: MAPFEnvironment
    dimx::Int64
    dimy::Int64
    obstacles::Set{Grid2DLocation}
    goals::Vector{Grid2DLocation}
    state_graph::SimpleVListGraph{Grid2DState}    = SimpleVListGraph{Grid2DState}()
    state_to_idx::Dict{Grid2DState,Int64}         = Dict{Grid2DState,Int64}()
    last_goal_constraint::Int64             = -1
    agent_idx::Int64                        = 0
    curr_goal_idx::Int64                    = 0
end


function get_mapf_state_from_idx(env::Grid2DEnvironment, idx::Int64)
    return env.state_graph.vertices[idx]
end

function get_mapf_action(env::Grid2DEnvironment, source::Int64, target::Int64)

    state1 = env.state_graph.vertices[source]
    state2 = env.state_graph.vertices[target]

    if equal_except_time(state1, state2)
        return Grid2DAction(Wait::Action)
    else
        if state2.x > state1.x
            return Grid2DAction(Right::Action)
        elseif state2.x < state1.x
            return Grid2DAction(Left::Action)
        elseif state2.y > state1.y
            return Grid2DAction(Up::Action)
        else
            return Grid2DAction(Down::Action)
        end
    end
    return nothing
end


function set_low_level_context!(env::Grid2DEnvironment, agent_idx::Int64, constraints::Grid2DConstraints)

    env.last_goal_constraint = -1
    env.agent_idx = agent_idx

    for vc in constraints.vertex_constraints
        if (vc.x == env.goals[agent_idx].x && vc.y == env.goals[agent_idx].y)
            env.last_goal_constraint = max(env.last_goal_constraint, vc.time)
        end
    end
end

function admissible_heuristic(env::Grid2DEnvironment, s::Grid2DState)
    return abs(s.x - env.goals[env.agent_idx].x) + abs(s.y - env.goals[env.agent_idx].y)
end

function is_solution(env::Grid2DEnvironment, s::Grid2DState)
    return s.x == env.goals[env.agent_idx].x && s.y == env.goals[env.agent_idx].y &&
            s.time > env.last_goal_constraint
end


function get_state(agent_idx::Int64, solution::Vector{PlanResult{Grid2DState,Grid2DAction,Int64}}, rel_time::Int64)

    @assert agent_idx <= length(solution)

    # t must be relative time index from start of solution
    if rel_time < length(solution[agent_idx].states)
        return solution[agent_idx].states[rel_time+1][1]
    end

    # Just return the last state after verifying it is not empty
    @assert ~(isempty(solution[agent_idx].states))
    return solution[agent_idx].states[end][1]

end

function state_valid(env::Grid2DEnvironment, constraints::Grid2DConstraints, s::Grid2DState)

    con = constraints.vertex_constraints

    return s.x >=0 && s.x < env.dimx && s.y >=0 && s.y < env.dimy &&
            ~(Grid2DLocation((x=s.x, y=s.y)) in env.obstacles) &&
            ~(VertexConstraint(time=s.time, x=s.x, y=s.y) in con)
end

function transition_valid(env::Grid2DEnvironment, constraints::Grid2DConstraints, s1::Grid2DState, s2::Grid2DState)

    con = constraints.edge_constraints

    return ~(EdgeConstraint(time=s1.time, x1=s1.x, y1=s1.y, x2=s2.x, y2=s2.y) in con)
end

function get_env_state_idx!(env::Grid2DEnvironment, s::Grid2DState)

    idx = get(env.state_to_idx, s, 0)

    if idx == 0
        add_vertex!(env.state_graph, s)
        nbr_idx = num_vertices(env.state_graph)
        env.state_to_idx[s] = nbr_idx
        idx = nbr_idx
    end

    return idx
end


function get_first_conflict(env::Grid2DEnvironment, solution::Vector{PlanResult{Grid2DState,Grid2DAction,Int64}})

    max_rel_time = 0
    for sol in solution
        max_rel_time = max(max_rel_time, length(sol.states) - 1)
    end

    for rel_time = 0:max_rel_time-1
        # Drive-drive vertex collisions
        for i = 1:length(solution)-1
            state1 = get_state(i, solution, rel_time)
            for j = i+1:length(solution)
                state2 = get_state(j, solution, rel_time)

                if equal_except_time(state1, state2)
                    return Grid2DConflict(time=rel_time, agent_1=i, agent_2=j,
                                    type=Vertex::ConflictType, x1=state1.x, y1=state1.y)
                end
            end
        end

        # Drive-drive edge
        for i = 1:length(solution)-1
            state1a = get_state(i, solution, rel_time)
            state1b = get_state(i, solution, rel_time + 1)

            for j = i+1:length(solution)
                state2a = get_state(j, solution, rel_time)
                state2b = get_state(j, solution, rel_time + 1)

                if equal_except_time(state1a, state2b) &&
                    equal_except_time(state1b, state2a)

                    return Grid2DConflict(time=rel_time, agent_1=i, agent_2=j,
                                    type=Edge::ConflictType, x1=state1a.x, y1=state1a.y,
                                    x2=state1b.x, y2=state1b.y)
                end
            end
        end
    end

    # No conflict
    return nothing
end

# Return a Dict{Int64, Grid2DConstraints}
function create_constraints_from_conflict(env::Grid2DEnvironment, conflict::Grid2DConflict)

    res_constraints = Dict{Int64, Grid2DConstraints}()

    if conflict.type == Vertex::ConflictType

        c1 = Grid2DConstraints()
        push!(c1.vertex_constraints, VertexConstraint(time=conflict.time, x=conflict.x1, y=conflict.y1))
        res_constraints[conflict.agent_1] = c1
        res_constraints[conflict.agent_2] = c1

    elseif conflict.type == Edge::ConflictType

        c1 = Grid2DConstraints()
        push!(c1.edge_constraints, EdgeConstraint(time=conflict.time, x1=conflict.x1, y1=conflict.y1,
                                                  x2=conflict.x2, y2=conflict.y2))
        res_constraints[conflict.agent_1] = c1

        c2 = Grid2DConstraints()
        push!(c2.edge_constraints, EdgeConstraint(time=conflict.time, x1=conflict.x2, y1=conflict.y2,
                                                  x2=conflict.x1, y2=conflict.y1))
        res_constraints[conflict.agent_2] = c2
    end

    return res_constraints
end



function low_level_search(env::Grid2DEnvironment, agent_idx::Int64, s::Grid2DState, constraints::Grid2DConstraints)

    # Reset env index
    env.curr_goal_idx = 0

    # Retrieve vertex index for state
    idx = get_env_state_idx!(env, s)

    # Set all edge weights to one
    edge_wt_fn(u, v) = 1

    # Set the heuristic
    heuristic(v) = admissible_heuristic(env, v)

    # Run the search
    # @info "RUNNING SEARCH!"
    vis = CBSGoalVisitorImplicit(env, constraints)
    a_star_states = a_star_light_shortest_path_implicit!(env.state_graph, edge_wt_fn,
                                                        idx, vis,
                                                        heuristic, Int64)

    plan_result = get_plan_result_from_astar(Grid2DState, Grid2DAction, Int64, env, a_star_states, idx, env.curr_goal_idx)

    return plan_result
end



mutable struct CBSGoalVisitorImplicit <: AbstractDijkstraVisitor
    env::Grid2DEnvironment
    constraints::Grid2DConstraints
end

function Graphs.include_vertex!(vis::CBSGoalVisitorImplicit, u::Grid2DState, v::Grid2DState,
                                d::N, nbrs::Vector{Int64}) where {N <: Number}

    if is_solution(vis.env, v)
        # @info "Found low-level solution!"
        vis.env.curr_goal_idx = get_env_state_idx!(vis.env, v)
        return false
    end

    # Need to generate neighbors of v using Actions
    empty!(nbrs)

    new_time = v.time + 1

    # Action = Wait
    temp_state = Grid2DState(time=new_time, x=v.x, y=v.y)
    if state_valid(vis.env, vis.constraints, temp_state) &&
        transition_valid(vis.env, vis.constraints, v, temp_state)
        nbr_idx = get_env_state_idx!(vis.env, temp_state)
        push!(nbrs, nbr_idx)
    end

    # Action = Left
    temp_state = Grid2DState(time=new_time, x=v.x-1, y=v.y)
    if state_valid(vis.env, vis.constraints, temp_state) &&
        transition_valid(vis.env, vis.constraints, v, temp_state)
        nbr_idx = get_env_state_idx!(vis.env, temp_state)
        push!(nbrs, nbr_idx)
    end

    # Action = Right
    temp_state = Grid2DState(time=new_time, x=v.x+1, y=v.y)
    if state_valid(vis.env, vis.constraints, temp_state) &&
        transition_valid(vis.env, vis.constraints, v, temp_state)
        nbr_idx = get_env_state_idx!(vis.env, temp_state)
        push!(nbrs, nbr_idx)
    end

    # Action = Up
    temp_state = Grid2DState(time=new_time, x=v.x, y=v.y+1)
    if state_valid(vis.env, vis.constraints, temp_state) &&
        transition_valid(vis.env, vis.constraints, v, temp_state)
        nbr_idx = get_env_state_idx!(vis.env, temp_state)
        push!(nbrs, nbr_idx)
    end

    # Action = Down
    temp_state = Grid2DState(time=new_time, x=v.x, y=v.y-1)
    if state_valid(vis.env, vis.constraints, temp_state) &&
        transition_valid(vis.env, vis.constraints, v, temp_state)
        nbr_idx = get_env_state_idx!(vis.env, temp_state)
        push!(nbrs, nbr_idx)
    end

    # @show nbrs
    return true
end
