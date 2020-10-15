"""
    get_coord_graph_from_state(env::MAPFEnvironment, curr_state::Vector{MAPFState})

"""
function get_coord_graph_from_state end

"""
    function get_coord_utility(env::MAPFEnvironment, s1::MAPFState, s2::MAPFState, a1::MAPFAction, a2::MAPFAction)
"""
function get_coord_utility end

@with_kw mutable struct CBSCGHighLevelNode{C <: Number}
    solution::Vector{PlanResult}
    cost::C
    id::Int64 = 0
    evaled_time::Int64 = 1
end

Base.isless(hln1::CBSCGHighLevelNode, hln2::CBSCGHighLevelNode) = hln1.cost < hln2.cost


# Phase 1 - Run standard CBS
# Phase 2 - Traverse solutions of agents, stopping when CG edge exists
# Can use CBSHighLevelNode as the data structure
# Create a CBSCGSolver that has CBS solver as an interim
# In second phase, will call message passing functions to resolve
# Role of second phase is whether to coordinate or not along paths
@with_kw mutable struct CBSCGSolver{E <: MAPFEnvironment}
    env::E
    phase1_cbs_solver::CBSSolver
    heap::MutableBinaryMinHeap{CBSCGHighLevelNode}
end

# TODO: Crucial function. Outline
# Step 1: Compute Dynamic CG at time t given current state
# Step 2: If CG, Do message passing as a function of alternative actions to get to next 
# already-planned state
# Step 3: If improvement from existing set of actions, propose change and compute new solution and cost and return
function adhoc_coord_along_path_at_time() end


function search!(solver::CBSCGSolver, initial_states::Vector{S}) where {S <: MAPF}

    num_agents = length(initial_states)

    # Bind environment to SOCC functions
    socc = SumOfCoordinatedCosts(solver.env, get_coord_graph_from_state, get_coord_utility)

    # Get normal CBS solution and compute its sum of coordinated costs
    phase1_soln, phase1_constr = search!(solver.phase1_cbs_solver, initial_states)
    phase1_soln_socc = compute_cost(socc, phase1_soln)

    # TODO: Eventually put a flag in compute_cost that checks if there is any
    # coordination to be done and if not then just return

    # Initialize phase 2 heap with solution node of phase 1 and eval time = 1
    start = CBSCGHighLevelNode(solution=phase1_soln, cost=phase1_soln_socc)

    push!(solver.heap, start)

    curr_id = 1

    while ~(isempty(solver.heap))

        # Get best node as per Sum of Coordinated Costs
        P = pop!(solver.heap)
        evaled_time = P.evaled_time

        # 
        last_action_times = sort(length(pr.actions) for pr in P.solution)
        if evaled_time > last_action_times[end-1]
            return P.solution
        end # returning


        # Determine if system state at time t has any CG improvement by message passing
        # If so, put in heap and break
        # NOTE: Ensure adhoc coord does not introduce new conflicts; use phase1_constr to guide
        change, new_soln, new_cost = adhoc_coord_along_path_at_time(phase1_constr)
        
        #$ new 
        if change
            new_node = CBSCGHighLevelNode(solution=new_soln, cost=new_cost, id=curr_id, evaled_time=evaled_time+1)
            curr_id += 1
            push!(solver.heap, new_node)
        end # change due to local coordination

        # Either way re-enter same node with time increased
        P.evaled_time += 1
        P.id = curr_id
        curr_id += 1
        push!(solver.heap, P)

    end # while heap not empty

end