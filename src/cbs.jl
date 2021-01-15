"""
    CBSHighLevelNode{S <: MAPFState, A <: MAPFAction, C <: Number, CNR <: MAPFConstraints}

Maintains the information for a high-level search tree node in CBS.

Attributes:
    - `solution::Vector{PlanResult{S,A,C}}` The current full solution encoded in the node
    - `constraints::Vector{CNR}` The set of constraints imposed on the node
    - `cost::C` The cost of the node's full solution
    - `id::Int64` A unique ID for the node in the high-level tree
"""
@with_kw mutable struct CBSHighLevelNode{S <: MAPFState, A <: MAPFAction, C <: Number, CNR <: MAPFConstraints}
    solution::Vector{PlanResult{S,A,C}} = Vector{PlanResult{S,A,C}}(undef, 0)
    constraints::Vector{CNR}            = Vector{CNR}(undef,0)
    cost::C                             = zero(C)
    id::Int64                           = 0
end

Base.isless(hln1::CBSHighLevelNode, hln2::CBSHighLevelNode) = hln1.cost < hln2.cost

# Functions required to be implemented by environment
"""
    set_low_level_context!(env::MAPFEnvironment, agent_idx::Int64, constraints::MAPFConstraints)

Update any contextual information before running the low-level search for an agent
"""
function set_low_level_context! end

"""
    get_first_conflict(env::MAPFEnvironment, solution::Vector{PlanResult})

Analyze the solution vector and return the first path-path conflict in it.
"""
function get_first_conflict end

"""
    create_constraints_from_conflict(env::MAPFEnvironment, conflict::MAPFConflict)

Given the MAPF conflict information, generate the corresponding low-level
search constraints on the individual agents.
"""
function create_constraints_from_conflict end

"""
    overlap_between_constraints(cbase::MAPFConstraints, cother::MAPFConstraints)

Return true if there is any overlap between the two constraint set arguments.
"""
function overlap_between_constraints end

"""
    add_constraint!(cbase::MAPFConstraints, cadd::MAPFConstraints)

Augment the set of constraints cbase in-place with new ones to add.
"""
function add_constraint! end

"""
    low_level_search!(solver::CBSSolver, agent_idx::Int64, s::MAPFState, constraints::MAPFConstraints)

Implement the actual low level search for a single agent on the environment. The search can be any of
the implicit A* variants in https://github.com/Shushman/Graphs.jl/tree/master/src
"""
function low_level_search! end


@with_kw mutable struct CBSSolver{S <: MAPFState, A <: MAPFAction, C <: Number, HC <: HighLevelCost,
                                  F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}
    env::E
    hlcost::HC                                                 = HC()
    heap::MutableBinaryMinHeap{CBSHighLevelNode{S,A,C,CNR}}    = MutableBinaryMinHeap{CBSHighLevelNode{S,A,C,CNR}}()
end

"""
search!(solver::CBSSolver{S,A,C,HC,F,CNR,E}, initial_states::Vector{S}) where {S <: MAPFState, A <: MAPFAction, C <: Number, HC <: HighLevelCost,
                                                                               F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}

    Calls the CBS Solver on the given problem.
"""
function search!(solver::CBSSolver{S,A,C,HC,F,CNR,E}, initial_states::Vector{S}) where {S <: MAPFState, A <: MAPFAction, C <: Number, HC <: HighLevelCost,
                                                                                    F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}

    num_agents = length(initial_states)

    start = CBSHighLevelNode{S,A,C,CNR}(solution = Vector{PlanResult{S,A,C}}(undef, num_agents),
                             constraints = Vector{CNR}(undef, num_agents))

    for idx = 1:num_agents

        start.constraints[idx] = get_empty_constraint(CNR)

        # Get constraints from low level context
        set_low_level_context!(solver.env, idx, start.constraints[idx])

        # Calls get_plan_result_from_astar within
        new_solution = low_level_search!(solver, idx, initial_states[idx], start.constraints[idx])

        # Return empty solution if cannot find
        if isempty(new_solution)
            return Vector{PlanResult{S,A,C}}(undef, 0)
        end

        start.solution[idx] = new_solution
    end

    # readline()
    # Insert start to heap
    start.cost = compute_cost(solver.hlcost, start.solution)
    push!(solver.heap, start)

    id = 1

    while ~(isempty(solver.heap))

        P = pop!(solver.heap)

        conflict = get_first_conflict(solver.env, P.solution)

        # If no conflict, we are done
        if conflict == nothing
            @info "SOLVED! Cost: ",P.cost
            # IMP - One or more solutions might be empty - need to check at higher level
            return P.solution
        end

        # Create additional nodes to resolve conflict (which is not nothing)
        constraints = create_constraints_from_conflict(solver.env, conflict)

        for constraint in constraints
            for (i, c) in constraint

                new_node = deepcopy(P)
                new_node.id = id

                # TODO: Check overlap??

                add_constraint!(new_node.constraints[i], c)
                # @debug new_node.constraints[i].vertex_constraints

                # Redo search with new constraint
                new_node.cost = deaccumulate_cost(solver.hlcost, new_node.cost, new_node.solution[i].cost)

                set_low_level_context!(solver.env, i, new_node.constraints[i])
                new_solution = low_level_search!(solver, i, initial_states[i], new_node.constraints[i])

                # readline()

                # Only create new node if we found a solution
                if ~(isempty(new_solution))

                    new_node.solution[i] = new_solution
                    new_node.cost = accumulate_cost(solver.hlcost, new_node.cost, new_solution.cost)
                    push!(solver.heap, new_node)
                    # @show new_node
                    id += 1
                end
            end
        end
    end

    # Return an empty solution
    return Vector{PlanResult{S,A,C}}(undef, 0)

end
