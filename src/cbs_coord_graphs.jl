"""
    get_coord_graph_from_state(env::MAPFEnvironment, curr_state::Vector{MAPFState})

"""
function get_coord_graph_from_state end

"""
    function get_coord_utility(env::MAPFEnvironment, s1::MAPFState, s2::MAPFState, a1::MAPFAction, a2::MAPFAction)
"""
function get_coord_utility end

# Phase 1 - Run standard CBS
# Phase 2 - Traverse solutions of agents, stopping when CG edge exists
# Can use CBSHighLevelNode as the data structure
# Create a CBSCGSolver that has CBS solver as an interim
# In second phase, will call message passing functions to resolve
@with_kw mutable struct CBSCGSolver{S <: MAPFState, A <: MAPFAction, C <: Number, HC <: HighLevelCost, 
                        F <: MAPFConflict, CNR <: MAPFConstraints, E <: MAPFEnvironment}
    env::E
    phase1_cbs_solver::CBSSolver{S,A,C,HC,F,CNR,E}
    heap::MutableBinaryMinHeap{CBSHighLevelNode{S,A,C,CNR}}
end

function search!(solver::CBSCGSolver, initial_states::Vector{S}) where {S <: MAPF}

    num_agents = length(initial_states)

    # Get normal CBS solution
    phase1_soln, phase1_constr = search!(solver.phase1_cbs_solver, initial_states)

    # Compute sum of coordinated costs from original soln
    # Bind environment to SOCC functions
    socc = SumOfCoordinatedCosts(solver.env, get_coord_graph_from_state, get_coord_utility)
    phase1_soln_socc = compute_cost(socc, phase1_soln)

    # Initialize phase 2 heap with solution node of phase 1
    start = CBSHighLevelNode(solution=phase1_soln, constraints=phase1_constr, cost=phase1_soln_socc)

    push!(solver.heap, start)

    id = 1

    while ~(isempty(solver.heap))

        

    end # while heap not empty

end