"""
    focal_heuristic(env::MAPFEnvironment, solution::Vector{PlanResult})

Computes the (potentially inadmissible) focal heuristic value for the high-level node
in ECBS; see the original ECBS paper for details - https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/view/8911
"""
function focal_heuristic end

# TODO: focal heuristics called on current solution?
@with_kw mutable struct ECBSHighLevelNode{S <: MAPFState, A <: MAPFAction, C <: Number, CNR <: MAPFConstraints}
    solution::Vector{PlanResult{S,A,C}} = Vector{PlanResult{S,A,C}}(undef, 0)
    constraints::Vector{CNR}            = Vector{CNR}(undef,0)
    cost::C                             = zero(C)
    lb::C                               = zero(C)
    focal_heuristic_value::C                   = zero(C)
    id::Int64                           = 0
end

Base.isless(hln1::ECBSHighLevelNode, hln2::ECBSHighLevelNode) = hln1.cost < hln2.cost

struct CompareFocalHeuristic
end

DataStructures.compare(comp::CompareFocalHeuristic, n1::ECBSHighLevelNode, n2::ECBSHighLevelNode) = (n1.focal_heuristic_value, n1.cost) < (n2.focal_heuristic_value, n2.cost)

@with_kw mutable struct ECBSSolver{S <: MAPFState, A <: MAPFAction, C <: Number, HC <: HighLevelCost, F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}
    env::E
    hlcost::HC                                                                      = HC()
    weight::Float64                                                                 = 1.0
    heap::MutableBinaryMinHeap{ECBSHighLevelNode{S,A,C,CNR}}                            = MutableBinaryMinHeap{ECBSHighLevelNode{S,A,C,CNR}}()
    hmap::Dict{Int,Int}                                                             = Dict{Int,Int}()
    focal_heap::MutableBinaryHeap{ECBSHighLevelNode{S,A,C,CNR},CompareFocalHeuristic}   = MutableBinaryHeap{ECBSHighLevelNode{S,A,C,CNR},CompareFocalHeuristic}()
    focal_hmap::Dict{Int,Int}                                                       = Dict{Int,Int}()
    num_global_conflicts::Int64                                                     = 0
end

"""
    search!(solver::ECBSSolver{S,A,C,HC,F,CNR,E}, initial_states::Vector{S}) where {S <: MAPFState, A <: MAPFAction, C <: Number, HC <: HighLevelCost,
                                                                               F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}

Calls the ECBS Solver on the given problem.
"""
function search!(solver::ECBSSolver{S,A,C,HC,F,CNR,E}, initial_states::Vector{S}) where {S <: MAPFState, A <: MAPFAction, C <: Number,  HC <: HighLevelCost,
                                                                                    F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}

    num_agents = length(initial_states)

    start = ECBSHighLevelNode{S,A,C,CNR}(solution = Vector{PlanResult{S,A,C}}(undef, num_agents),
                              constraints = Vector{CNR}(undef, num_agents))

    for idx = 1:num_agents

        start.constraints[idx] = get_empty_constraint(CNR)

        # Get constraints from low level context
        set_low_level_context!(solver.env, idx, start.constraints[idx])

        # Calls get_plan_result_from_astar within
        new_solution = low_level_search!(solver, idx, initial_states[idx], start.constraints[idx],
                                         Vector{PlanResult{S,A,C}}(undef, 0))

        # Return empty solution if cannot find
        if isempty(new_solution)
          return Vector{PlanResult{S,A,C}}(undef, 0)
        end

        start.solution[idx] = new_solution
        start.lb += start.solution[idx].fmin
    end
    start.cost = compute_cost(solver.hlcost, start.solution)
    start.focal_heuristic_value = focal_heuristic(solver.env, start.solution)

    # Push on to heaps
    solver.hmap[start.id] = push!(solver.heap, start)
    solver.focal_hmap[start.id] = push!(solver.focal_heap, start)

    best_cost = start.cost

    id = 1

    while ~(isempty(solver.heap))

        old_best_cost = best_cost
        best_cost = top(solver.heap).cost

        if best_cost > old_best_cost

            for node in sort(solver.heap.nodes, by = x->x.value.cost)
                val = node.value.cost

                if val > solver.weight * old_best_cost && val <= solver.weight * best_cost &&
                    ~(haskey(solver.focal_hmap, node.value.id))
                    solver.focal_hmap[node.value.id] = push!(solver.focal_heap, node.value)
                end

                if val > solver.weight * best_cost
                    break
                end
            end
        end

        # TODO : Check consistency?

        # Pop from focal list and do rest
        # @show solver.focal_heap
        focal_entry, focal_handle = top_with_handle(solver.focal_heap)
        heap_handle = solver.hmap[focal_entry.id]

        # @show focal_entry.id

        pop!(solver.focal_heap)
        delete!(solver.focal_hmap, focal_entry.id)
        delete!(solver.heap, heap_handle)
        delete!(solver.hmap, focal_entry.id)

        # Handle conflict
        conflict = get_first_conflict(solver.env, focal_entry.solution)

        # If no conflict, we are done
        if conflict == nothing
            @info "SOLVED! Cost: ",focal_entry.cost
            # IMP - One or more solutions might be empty - need to check at higher level
            return focal_entry.solution
        end

        solver.num_global_conflicts += 1

        # VECTOR OF CONSTRAINTS
        constraints = create_constraints_from_conflict(solver.env, conflict)

        for constraint in constraints
            for (i, c) in constraint

                # @show (i, c)

                new_node = deepcopy(focal_entry)


                add_constraint!(new_node.constraints[i], c)

                new_node.cost = deaccumulate_cost(solver.hlcost, new_node.cost, new_node.solution[i].cost)
                new_node.lb -= new_node.solution[i].fmin

                set_low_level_context!(solver.env, i, new_node.constraints[i])
                new_solution = low_level_search!(solver, i, initial_states[i], new_node.constraints[i], new_node.solution)

                if ~(isempty(new_solution))

                    # Can enter a new node
                    new_node.id = id
                    new_node.solution[i] = new_solution
                    new_node.cost = accumulate_cost(solver.hlcost, new_node.cost, new_solution.cost)
                    new_node.lb += new_solution.fmin
                    new_node.focal_heuristic_value = focal_heuristic(solver.env, new_node.solution)

                    solver.hmap[id] = push!(solver.heap, new_node)

                    if new_node.cost <= solver.weight * best_cost
                        solver.focal_hmap[id] = push!(solver.focal_heap, new_node)
                    end
                    id += 1
                end
            end
        end
    end

    # Return an empty solution
    return Vector{PlanResult{S,A,C}}(undef, 0)

end
