abstract type MAPFCoordinationStrategy end

"""
    function coord_graph_from_states(env::MAPFEnvironment, curr_states::Vector{MAPFState}, coordination_constraints::Vector{Set{Int64}}, agents_to_exclude::Set{Int64})

    Depending on the domain we should restrict the degree of nodes so that capacity constraints (if any) are not violated. Also don't make edges if coord constraints.
"""
function coord_graph_from_states end

"""
    function coord_cost(env::MAPFEnvironment, s1::MAPFState, s2::MAPFState, a1::MAPFAction, a2::MAPFAction, constraints::Vector{MAPFConstraints})

Have to think very carefully about this to avoid over/undercounting
"""
function coord_cost end

"""
    function all_coord_strategies(env::MAPFEnvironment, curr_states::Vector{MAPFState}, curr_cg::SimpleGraph, idx::Int64)

All possible coordination strategies at the current state.
"""
function all_coord_strategies end

function compute_updated_path_with_coordination end

function phase1_soln_constraints end

function agent_cost end

"""
    agent_evaluation_timesteps: The timestep up to which each agent has been evaluated for coordination
"""
@with_kw mutable struct CBSCGHighLevelNode{C <: Number}
    solution::Vector{PlanResult}
    cost::C
    agent_evaluation_timesteps::Vector{Int64}
    coordination_constraints::Vector{Set{Int64}}
    id::Int64 = 0
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
    heap::MutableBinaryMinHeap{CBSCGHighLevelNode}
    message_rounds::Int64
end

struct CoordinatedPathData{C <: Number}
    solution::PlanResult
    coordination_time::Int64
end

"""
    If coordination possible, consider it if beneficial, return the updated paths, cost, evaluation timesteps, and coordination constraints.
"""
function adhoc_coord_along_path_at_time(solver::CBSCGSolver,
                                        P::CBSCGHighLevelNode{C},
                                        phase1_constraints::Vector{CNR},
                                        eval_time::Int64) where {C <: Number, CNR <: MAPFConstraints}
    
    # Extract current states of agents
    num_agents = length(P.solution)
    curr_states = [P.solution[i].states[min(eval_time, end)][1] for i = 1:num_agents]

    # Exclude from CG consideration (set deg to 0) any agents that are ahead of eval time or done
    agents_to_exclude = Set{Int64}()
    for i = 1:num_agents
        if eval_time < P.agent_evaluation_timesteps[i] || eval_time >= length(P.solution[i].states)
            push!(agents_to_exclude, i)
        end
    end

    # Get current CG for state; we will filter out for eval time here
    curr_cg = coord_graph_from_states(solver.env, curr_states, P.coordination_constraints, agents_to_exclude)  
    
    # Nothing to coordinate
    if ne(curr_cg) == 0
        return false, P
    end

    # For agents that can possibly coordinate, compute the cost of the rest of their current paths
    cg_agent_rest_path_costs = Dict{Int64,C}()
    total_cg_agent_rest_path_costs = zero(C)
    for v in vertices(curr_cg)
        if degree(curr_cg, v) > 0
            agent_rest_path_cost = P.solution[v].states[end][2] - P.solution[v].states[eval_time][2]
            cg_agent_rest_path_costs[v] = agent_rest_path_cost
            total_cg_agent_rest_path_costs += agent_rest_path_cost
        end
    end # v in vertices

    ## MESSAGE PASSING

    # Need to set up matrix for messages
    agent_coords = [all_coord_strategies(solver.env, curr_states, curr_cg, n) for n = 1:num_agents]
    n_max_coords = maximum(length(aa) for aa in agent_coords)

    # Again, if no coord to be done, move on
    if n_max_coords == 0
        return false, P.solution, P.cost
    end

    n_edges = ne(tree.coordination_stats.adjmatgraph)
    fwd_messages = zeros(C, n_edges, n_max_coords+1)
    bwd_messages = zeros(C, n_edges, n_max_coords+1)

    # TODO: For each edge, compute the benefit of each coord strategy to indiv agent and to pair
    # Also need to track the actual recomputed solns
    # This is the key low-level routine that domain must implement efficiently
    # NOTE: We use n_max_coords + 1 to account for the null coord strategy in maxplus
    per_agent_coord_costs = zeros(C, num_agents, n_max_coords+1)
    coord_payoff_cost = zeros(C, num_agents, num_agents, n_max_coords+1, n_max_coords+1)
    
    agent_coord_paths = Dict{Int64,Dict}()
    
    # First enter per_agent_coord costs for those that are in cg
    for (agent, pathcost) in cg_agent_rest_path_costs
        per_agent_coord_costs[agent][1] = pathcost
        agent_coord_paths[agent] = Dict{Int64,CoordinatedPathData}()
        for (idx, coord) in enumerate(all_coord_strategies[agent])
            updated_soln = compute_updated_path_with_coordination(solver.env, curr_states,
                                                        agent, coord, phase1_constraints, P.solution, eval_time)
            agent_coord_paths[idx+1] = updated_soln
            per_agent_coord_costs[agent][idx+1] = updated_soln.solution.cost
        end
    end

    # Now enter coordination payoffs for coord strategies
    for (e_idx, e) in enumerate(edges(curr_cg))

        i = e.src
        j = e.dst
        
        # If both agents have options to coordinate
        if length(all_coord_strategies[i])*length(all_coord_strategies[j]) > 0

            for (ci_idx, ci) in enumerate(all_coord_strategies[i])
                for (cj_idx, cj) in enumerate(all_coord_strategies[j])
                    
                    if compatible_strategies(solver.env, curr_states, i, j, ci, cj) == true

                        @assert agent_coord_paths[i][ci_idx+1].coordination_time == agent_coord_paths[j][cj_idx+1].coordination_time

                        path_i = agent_coord_paths[i][ci_idx+1].solution
                        path_j = agent_coord_paths[j][cj_idx+1].solution

                        ij_coord_payoff = compute_coordination_payoff_paths(solver.env, ci, cj, path_i, path_j,
                                                                eval_time, agent_coord_paths[i][ci_idx+1].coordination_time)

                        coord_payoff_cost[i, j, ci_idx+1, cj_idx+1] = ij_coord_payoff
                        coord_payoff_cost[j, i, cj_idx+1, ci_idx+1] = ij_coord_payoff

                    end
                end
            end
        end
    end


    # We're trying to minimize with message passing
    for r = 1:solver.message_rounds

        # Copy over old messages
        fwd_messages_old = deepcopy(fwd_messages)
        bwd_messages_old = deepcopy(bwd_messages)

        # Iterate over edges
        for (e_idx, e) in enumerate(edges(curr_cg))

            i = e.src
            j = e.dst

            # Forward messages
            for aj_idx = 1:length(agent_coords[j])+1

                fwd_message_vals = zeros(C, length(agent_coords[i])+1)

                for ai_idx = 1:length(agent_coords[i])+1
                    fwd_message_vals[ai_idx] = per_agent_coord_costs[i, ai_idx] - bwd_messages_old[e_idx, ai_idx] + coord_payoff_cost[i, j, ai_idx, aj_idx]
                end

                fwd_messages[e_idx, aj_idx] = minimum(fwd_message_vals)
            end

            # Backward messages
            for ai_idx = 1:length(agent_coords[i])+1

                bwd_message_vals = zeros(C, length(agent_coords[j])+1)

                for aj_idx = 1:length(agent_coords[j])+1
                    bwd_message_vals[aj_idx] = per_agent_coord_costs[j, aj_idx] - fwd_messages_old[e_idx,aj_idx] + coord_payoff_cost[i, j, ai_idx, aj_idx]
                end

                bwd_messages[e_idx, ai_idx] = minimum(bwd_message_vals)
            end
        end
    
        # TODO : Consider message normalization later?

        fnormdiff = norm(fwd_messages - fwd_messages_old)
        bnormdiff = norm(bwd_messages - bwd_messages_old)

        # Update agent_costs for nonzero deg nodes
        # for n in keys(cg_agent_rest_path_costs)

        #     nbrs = neighbors(curr_cg, n)
        #     edgelist = collect(edges(adjgraphmat))

        #     for nbr in nbrs
        #         if Edge(n, nbr) in edgelist
        #             agent_costs[n, :] += bwd_messages[findfirst(isequal(Edge(n, nbr)), edgelist), :]
        #         elseif Edge(nbr, n) in edgelist
        #             agent_costs[n, :] += fwd_messages[findfirst(isequal(Edge(nbr, n)), edgelist), :]
        #         end
        #     end

        # end # go over nonzero degree nodes

        # If converged, break
        if isapprox(fnormdiff, zero(C)) && isapprox(bnormdiff, zero(C))
            break
        end
    end # r = 1:rounds

    # Go over nonzero deg nodes and update
    new_node = CBSCGHighLevelNode(solution=P.solution, cost=P.cost,
                    agent_evaluation_timesteps=P.agent_evaluation_timesteps,
                    coordination_constraints=P.coordination_constraints)
    
    # TODO: Update solution, cost, eval timesteps and coord constraints

    for (n, pathcost) in cg_agent_rest_path_costs


        nbrs = neighbors(curr_cg, n)
        edgelist = collect(edges(curr_cg))

        for nbr in nbrs
            if Edge(n, nbr) in edgelist
                per_agent_coord_costs[n, :] += bwd_messages[findfirst(isequal(Edge(n, nbr)), edgelist), :]
            elseif Edge(nbr, n) in edgelist
                per_agent_coord_costs[n, :] += fwd_messages[findfirst(isequal(Edge(nbr, n)), edgelist), :]
            end
        end


        cc, idx = findmin(per_agent_costs[n, 1:length(agent_coords[n])+1])   # Only choose up to agent ID
        
        
    end

    # If overall coordination chosen, modify solution
    if cg_vert_coord_costs < cg_vert_action_costs
        new_cost = cost + cg_vert_coord_costs
        return true, new_soln, new_cost
    else
        return false, solution, cost
    end
end



function search!(solver::CBSCGSolver, initial_states::Vector{S}) where {S <: MAPFState}

    num_agents = length(initial_states)

    # Bind environment to SOCC functions
    socc = SumOfCoordinatedCosts(solver.env, get_coord_graph_from_state, coord_cost)

    # Get normal CBS solution and compute its sum of coordinated costs
    phase1_soln, phase1_constr = phase1_soln_constraints(solver.env, initial_states)
    phase1_soln_socc = compute_cost(socc, phase1_soln)

    # TODO: Eventually put a flag in compute_cost that checks if there is any
    # coordination to be done and if not then just return

    # Initialize phase 2 heap with solution node of phase 1 and eval time = 1
    start = CBSCGHighLevelNode(solution=phase1_soln, cost=phase1_soln_socc,
                               agent_evaluation_timesteps = ones(Int64, num_agents),
                               coordination_constraints = [Set{Int64}() for _ = 1:num_agents])

    push!(solver.heap, start)

    curr_id = 1

    while ~(isempty(solver.heap))

        # Get best node as per Sum of Coordinated Costs
        P = pop!(solver.heap)

        eval_done = true
        for (pr, evt) in zip(P.agent_evaluation_timesteps, P.solution)
            if evt < length(pr.states)
                eval_done = false
                break
            end
        end

        if eval_done
            return P.solution
        end

        eval_time = minimum(P.agent_evaluation_timesteps)


        # Determine if system state at time t has any CG improvement by message passing
        # If so, put in heap and break
        # NOTE: Ensure adhoc coord does not introduce new conflicts; use phase1_constr to guide
        change, new_node_info = adhoc_coord_along_path_at_time(solver.env, P, phase1_constr, eval_time)
        
        if change
            new_node = CBSCGHighLevelNode(solution=new_node_info.solution,
                            cost=new_node_info.cost,
                            agent_evaluation_timesteps=new_node_info.agent_evaluation_timesteps,
                            coordination_constraints = new_node_info.coordination_constraints,
                            id=curr_id)
            curr_id += 1
            push!(solver.heap, new_node)

            # TODO: Only if there is a change or either way?
            P.coordination_constraints = new_node_info.coordination_constraints
        end # change due to local coordination

        # Either way re-enter same node with time increased
        P.agent_evaluation_timesteps = P.agent_evaluation_timesteps .+ 1

        P.id = curr_id
        curr_id += 1
        push!(solver.heap, P)

    end # while heap not empty

    return PlanResult{S,A,C}[], CNR[]

end