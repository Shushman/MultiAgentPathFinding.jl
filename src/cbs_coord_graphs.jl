"""
    get_coord_graph_from_state(env::MAPFEnvironment, curr_state::Vector{MAPFState})

"""
function get_coord_graph_from_state end

"""
    function get_coord_cost(env::MAPFEnvironment, s1::MAPFState, s2::MAPFState, a1::MAPFAction, a2::MAPFAction, constraints::Vector{MAPFConstraints})

Have to think very carefully about this to avoid over/undercounting
"""
function get_coord_cost end

"""
    function get_actions_between_states(env::MAPFEnvironment, s1::MAPFState, s2::MAPFEnvironment)

Return all possible actions that can take agent from s1 to s2 (including coordination). If s1 == s2, return empty vector
"""
function get_all_mapf_actions_between_states end

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
    message_rounds::Int64
end

# TODO: Crucial function. Outline
# Step 1: Compute Dynamic CG at time t given current state
# Step 2: If CG, Do message passing as a function of alternative actions to get to next 
# already-planned state
# Step 3: If improvement from existing set of actions, propose change and compute new solution and cost and return
function adhoc_coord_along_path_at_time(solver::CBSCGSolver,
                                        solution::Vector{PR}, cost::C,
                                        curr_time::Int64) where {C <: Number, PR <: PlanResult}
    
    # Extract current state
    num_agents = length(solution)
    state_type = typeof(solution[1].states[1][1]) 
    curr_state = Vector{state_type}(undef, num_agents)
    for i = 1:num_agents
        curr_state[i] = solution[i].states[min(curr_time, end)][1]
    end

    # Get current CG for state
    curr_cg = get_coord_graph_from_state(solver.env, curr_state)

    # Add up action costs for zero-degree and nz-deg CG nodes
    nonzero_deg_nodes = Set{Int64}()
    zero_deg_action_costs = zero(C)
    cg_vert_action_costs = zero(C)
    for v in vertices(curr_cg)
        if curr_time <= length(solution[v].actions)
            if degree(curr_cg, v) == 0           
                zero_deg_action_costs += solution[v].actions[curr_time][2]
            else
                push!(nonzero_deg_nodes, v)
                cg_vert_action_costs += solution[v].actions[curr_time][2]
            end
        end
    end # v in vertices

    if cg_vert_action_costs == zero(C)
        return false, solution, cost
    end

    ## MESSAGE PASSING

    # Need to set up matrix for messages
    agent_actions = [get_all_mapf_actions_between_states(env,
                     solution[n].states[min(end, curr_time)],
                     solution[n].states[min(end, curr_time+1)]) for n = 1:num_agents]
    n_max_actions = maximum(length(aa) for aa in agent_actions)
    n_edges = ne(tree.coordination_stats.adjmatgraph)

    fwd_messages = zeros(C, n_edges, n_max_actions)
    bwd_messages = zeros(C, n_edges, n_max_actions)

    # We'll only care about the agents in CG
    agent_costs = zeros(C, num_agents, n_max_actions)

    # We're trying to minimize with message passing as all costs are positive
    for r = 1:solver.message_rounds

        # Copy over old messages
        fwd_messages_old = deepcopy(fwd_messages)
        bwd_messages_old = deepcopy(bwd_messages)

        # Iterate over edges
        for (e_idx, e) in enumerate(edges(curr_cg))

            i = e.src
            j = e.dst

            # Forward messages
            for (aj_idx, aj) in enumerate(agent_actions[j])

                fwd_message_vals = zeros(C, length(agent_actions[i]))

                for (ai_idx, ai) in enumerate(agent_actions[i])
                    fwd_message_vals[ai_idx] = agent_costs[i, ai_idx] - bwd_messages_old[e_idx,ai_idx] + get_coord_cost(solver.env, curr_state[i], curr_state[j], ai, aj)
                end # iter over agent_actions[i]
                fwd_messages[e_idx, aj_idx] = minimum(fwd_message_vals)
            end # iter over agent_actions[j]

            # Backward messages
            for (ai_idx, ai) in enumerate(agent_actions[i])

                bwd_message_vals = zeros(C, length(agent_actions[j]))

                for (aj_idx, aj) in enumerate(agent_actions[j])
                    bwd_message_vals[aj_idx] = agent_costs[j, aj_idx] - fwd_messages_old[e_idx,aj_idx] + get_coord_cost(solver.env, curr_state[i], curr_state[j], ai, aj)
                end # iter over agent_actions[j]
                bwd_messages[e_idx, ai_idx] = minimum(bwd_message_vals)
            end # iter over agent_actions[i]
        end
    
        # TODO : Consider message normalization later?

        fnormdiff = norm(fwd_messages - fwd_messages_old)
        bnormdiff = norm(bwd_messages - bwd_messages_old)

        # Update agent_costs for nonzero deg nodes
        for n in nonzero_deg_nodes

            nbrs = neighbors(curr_cg, n)
            edgelist = collect(edges(adjgraphmat))

            for nbr in nbrs
                if Edge(n, nbr) in edgelist
                    agent_costs[n, :] += bwd_messages[findfirst(isequal(Edge(n, nbr)), edgelist), :]
                elseif Edge(nbr, n) in edgelist
                    agent_costs[n, :] += fwd_messages[findfirst(isequal(Edge(nbr, n)), edgelist), :]
                end
            end

        end # go over nonzero degree nodes

        # If converged, break
        if isapprox(fnormdiff, zero(C)) && isapprox(bnormdiff, zero(C))
            break
        end
    end # r = 1:rounds

    # Go over nonzero deg nodes and update
    new_soln = deepcopy(solution)
    cg_vert_coord_costs = zero(C) # MUST be either zero (don't coord) or negative at the end
    for n in nonzero_deg_nodes
        cc, idx = findmin(agent_costs[n, :])
        cg_vert_coord_costs += cc
        # Update action, cost of agent 
        new_soln[n].actions[curr_time] = (agent_actions[n, idx], solution[n].actions[curr_time][2] + cc)
    end

    @assert cg_vert_coord_costs <= zero(C) "Message passing messed up!"

    # If overall coordination chosen, modify soln
    if cg_vert_coord_costs < zero(C)
        new_cost = cost + cg_vert_coord_costs
        return true, new_soln, new_cost
    else
        return false, solution, cost
    end
end



function search!(solver::CBSCGSolver, initial_states::Vector{S}) where {S <: MAPFState}

    num_agents = length(initial_states)

    # Bind environment to SOCC functions
    socc = SumOfCoordinatedCosts(solver.env, get_coord_graph_from_state, get_coord_cost)

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

        # Return solution if it has been evaluated throughout
        last_action_times = sort(length(pr.actions) for pr in P.solution)
        if evaled_time > last_action_times[end-1]
            return P.solution
        end # returning


        # Determine if system state at time t has any CG improvement by message passing
        # If so, put in heap and break
        # NOTE: Ensure adhoc coord does not introduce new conflicts; use phase1_constr to guide
        change, new_soln, new_cost = adhoc_coord_along_path_at_time(solver.env, P.solution,
                                                                    P.cost, evaled_time)
        
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

    return PlanResult{S,A,C}[], CNR[]

end