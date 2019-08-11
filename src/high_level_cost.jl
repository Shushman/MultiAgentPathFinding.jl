abstract type HighLevelCost end

function compute_cost end
function accumulate_cost! end
function deaccumulate_cost! end


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
