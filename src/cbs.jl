@with_kw mutable struct CBSHighLevelNode{S <: MAPFState, A <: MAPFAction, C <: Number, CNR <: MAPFConstraints}
    solution::Vector{PlanResult{S,A,C}} = Vector{PlanResult{S,A,C}}(undef, 0)
    constraints::Vector{CNR}            = Vector{CNR}(undef,0)
    cost::C                             = zero(C)
    id::Int64                           = 0
end

Base.isless(hln1::CBSHighLevelNode, hln2::CBSHighLevelNode) = hln1.cost < hln2.cost

# Functions required to be implemented by environment
function set_low_level_context! end
function get_first_conflict end
function create_constraints_from_conflict end
function overlap_between_constraints end
function add_constraint! end
function low_level_search! end



@with_kw mutable struct CBSSolver{S <: MAPFState, A <: MAPFAction, C <: Number,
                                  F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}
    env::E
    heap::MutableBinaryMinHeap{CBSHighLevelNode{S,A,C,CNR}}    = MutableBinaryMinHeap{CBSHighLevelNode{S,A,C,CNR}}()
end


function search!(solver::CBSSolver{S,A,C,F,CNR,E}, initial_states::Vector{S}) where {S <: MAPFState, A <: MAPFAction, C <: Number,
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
        # @show idx
        # @show new_solution.states

        # Return empty solution if cannot find
        if isempty(new_solution)
            return Vector{PlanResult{S,A,C}}(undef, 0)
        end

        start.solution[idx] = new_solution
        start.cost += start.solution[idx].cost
    end

    # readline()
    # Insert start to heap
    push!(solver.heap, start)

    id = 1

    while ~(isempty(solver.heap))

        P = pop!(solver.heap)
        # @show P
        # on_expand_high_level_node(solver, P.cost)

        conflict = get_first_conflict(solver.env, P.solution)
        # @show conflict

        # If no conflict, we are done
        if conflict == nothing
            @info "SOLVED! Cost: ",P.cost
            # IMP - One or more solutions might be empty - need to check at higher level
            return P.solution
        end

        # Create additional nodes to resolve conflict (which is not nothing)
        constraints = create_constraints_from_conflict(solver.env, conflict)
        # @show constraints

        for (i, c) in constraints

            new_node = deepcopy(P)
            new_node.id = id

            # TODO: Check overlap??

            add_constraint!(new_node.constraints[i], c)
            # @debug new_node.constraints[i].vertex_constraints

            # Redo search with new constraint
            new_node.cost -= new_node.solution[i].cost

            set_low_level_context!(solver.env, i, new_node.constraints[i])
            new_solution = low_level_search!(solver, i, initial_states[i], new_node.constraints[i])
            # @show i
            # @show new_solution.states

            # readline()

            # Only create new node if we found a solution
            if ~(isempty(new_solution))

                new_node.solution[i] = new_solution
                new_node.cost += new_solution.cost
                push!(solver.heap, new_node)
                # @show new_node
            end

            id += 1
        end
    end

    # Return an empty solution
    return Vector{PlanResult{S,A,C}}(undef, 0)

end
