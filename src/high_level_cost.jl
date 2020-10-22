""" HighLevelCost

Abstract base type for the two kinds of high-level MAPF costs - Makespan and Sum-of-Costs
"""
abstract type HighLevelCost end

"""
    compute_cost(::HighLevelCost, solution::Vector{PR}) where {PR <: PlanResult}

Compute the cost for the MAPF solution.
"""
function compute_cost end

"""
    accumulate_cost(::HighLevelCost, old_cost::C, new_entry_cost::C) where {C <: Number}

Given a new entry, update the high-level cost according to the metric (addition for sum-of-costs,
maximum for makespan)
"""
function accumulate_cost end

"""
    deaccumulate_cost(::HighLevelCost, old_cost::C, new_entry_cost::C) where {C <: Number}

Given an entry to be removed, update the high-level cost according to the metric (subtraction for sum-of-costs,
minimum for makespan)
"""
function deaccumulate_cost end


struct SumOfCosts <: HighLevelCost
end

# Assumes that all entries are valid
function compute_cost(::SumOfCosts, solution::Vector{PR}) where {PR <: PlanResult}
    return sum(s.cost for s in solution)
end

function accumulate_cost(::SumOfCosts, old_cost::C, new_entry_cost::C) where {C <: Number}
    return (old_cost + new_entry_cost)
end

function deaccumulate_cost(::SumOfCosts, old_cost::C, new_entry_cost::C) where {C <: Number}
    return (old_cost - new_entry_cost)
end


struct Makespan <: HighLevelCost
end

# Assumes that all entries are valid
function compute_cost(::Makespan, solution::Vector{PR}) where {PR <: PlanResult}
    return maximum(s.cost for s in solution)
end

function accumulate_cost(::Makespan, old_cost::C, new_entry_cost::C) where {C <: Number}
    return max(old_cost, new_entry_cost)
end

function deaccumulate_cost(::Makespan, old_cost::C, new_entry_cost::C) where {C <: Number}
    return min(old_cost, new_entry_cost)
end



## New cost type for CBS-CG
## Compute sum of individual paths but if two states have an edge 
## then compute the pairwise costs of their next actions
struct SumOfCoordinatedCosts <: HighLevelCost
    env::MAPFEnvironment
    get_coord_graph_from_state::Function   # Maps set of states to CG over agents
    get_coord_cost::Function             # Maps pair of states and actions to utility (can be negative)
end

# Non-trivial: iterate through time-steps of the solution
# Compute the coordination graph for all agents (use the final state of finished agent)
# Add up action costs and if CG edge then add utility cost
function compute_cost(socc::SumOfCoordinatedCosts, solution::Vector{PR},
                      start_time::Int64=1) where {PR <: PlanResult}

    # TODO : Verify this sort is okay
    finish_times = sort((length(pr.states), i) for (i, pr) in enumerate(solution))
    second_last_finish = finish_times[end-1][1]
    state_type = typeof(solution[1].states[1][1])   # A bit hacky
    nagents = length(solution)

    total_cost = zero(typeof(solution[1].states[1][2]))

    # Continue up to the last action of the second last agent 
    for t = start_time:second_last_finish-1
        curr_state = Vector{state_type}(undef, nagents)
        for i = 1:nagents
            curr_state[i] = solution[i].states[min(t, end)][1]  # Get the current or final state of agent i
        end
    
        # Get the current CG
        current_cg = socc.get_coord_graph_from_state(socc.env, curr_state)

        action_counted_agents = Set{Int64}()
        for edge in edges(current_cg)

            # Only consider edges between agents in play
            # which still have actions to do at time t
            if t <= min(length(solution[edge.src].actions), length(solution[edge.dst].actions))

                if ~(edge.src in action_counted_agents)
                    push!(action_counted_agents, edge.src)
                    total_cost += indiv_agent_cost(socc.env, edge.src, curr_state[edge.src], solution[edge.src].actions[t][2])
                end
                if ~(edge.dst in action_counted_agents)
                    push!(action_counted_agents, edge.dst)
                    total_cost += indiv_agent_cost(socc.env, edge.dst, curr_state[edge.dst], solution[edge.dst].actions[t][2])
                end

                # NOTE: Utility is a SEPARATE addition or reduction or no-effect
                total_cost += socc.get_coord_cost(socc.env, curr_state[edge.src], curr_state[edge.dst],
                                                     solution[edge.src].actions[t], solution[edge.dst].actions[t])
            end # t < min(vertex action sets)
        end # edge in edges(curr_cg)

        # For all agents that have not been counted that have actions remaining
        # Count the cost of their actions
        for i = 1:nagents
            if ~(i in action_counted_agents) && t <= length(solution[i].actions)
                total_cost += solution[i].actions[t][2]
            end
        end
    end

    # Now just count the actions of the last unfinished agent
    (last_finish, last_agent) = finish_times[end]
    for t = second_last_finish:last_finish-1
        total_cost += solution[last_agent].actions[t]
    end
    
    return total_cost
end

# NOTE: No accumulation/deaccumulation for SOCC (only a function of full solution). But that's okay