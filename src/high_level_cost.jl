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
