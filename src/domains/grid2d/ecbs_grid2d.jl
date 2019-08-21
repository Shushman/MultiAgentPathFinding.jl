# Only need to define heuristic stuff
function focal_state_heuristic_grid2d(env::Grid2DEnvironment, solution::Vector{PlanResult{Grid2DState,Grid2DAction,Int64}},
                                      s::Grid2DState)

    num_conflicts = 0

    for (i, agent_soln) in enumerate(solution)
        if i != env.agent_idx && ~(isempty(agent_soln))
            state2 = get_state(i, solution, s.time)
            if equal_except_time(s, state2)
                num_conflicts += 1
            end
        end
    end

    return num_conflicts
end


function focal_transition_heuristic_grid2d(env::Grid2DEnvironment,  solution::Vector{PlanResult{Grid2DState,Grid2DAction,Int64}},
                                           s1a::Grid2DState, s1b::Grid2DState)

    num_conflicts = 0

    for (i, agent_soln) in enumerate(solution)
        if i != env.agent_idx && ~(isempty(agent_soln))

            s2a = get_state(i, solution, s1a.time)
            s2b = get_state(i, solution, s1b.time)

            if equal_except_time(s1a, s2b) && equal_except_time(s1b, s2a)
                num_conflicts += 1
            end
        end
    end

    return num_conflicts
end

function focal_heuristic(env::Grid2DEnvironment, solution::Vector{PlanResult{Grid2DState,Grid2DAction,Int64}})

    num_conflicts = 0
    max_rel_time = 0

    for sol in solution
        max_rel_time = max(max_rel_time, length(sol.states) - 1)
    end

    for rel_time = 0:max_rel_time-1
        # Drive-drive vertex collisions
        for i = 1:length(solution)-1
            state1 = get_state(i, solution, rel_time)
            for j = i+1:length(solution)
                state2 = get_state(j, solution, rel_time)

                if equal_except_time(state1, state2)
                    num_conflicts += 1
                end
            end
        end

        # Drive-drive edge
        for i = 1:length(solution)-1
            state1a = get_state(i, solution, rel_time)
            state1b = get_state(i, solution, rel_time + 1)

            for j = i+1:length(solution)
                state2a = get_state(j, solution, rel_time)
                state2b = get_state(j, solution, rel_time + 1)

                if equal_except_time(state1a, state2b) &&
                    equal_except_time(state1b, state2a)

                    num_conflicts += 1
                end
            end
        end
    end

    return num_conflicts
end

function low_level_search!(solver::ECBSSolver, agent_idx::Int64, s::Grid2DState,
                           constraints::Grid2DConstraints, solution::Vector{PlanResult{Grid2DState,Grid2DAction,Int64}})

    env = solver.env

    env.curr_goal_idx = 0

    idx = get_env_state_idx!(env, s)

    edge_wt_fn(u, v) = 1

    # Set the heuristics
    admissible_heuristic(s) = admissible_heuristic_grid2d(solver.env, s)
    focal_state_heuristic(s) = focal_state_heuristic_grid2d(solver.env, solution, s)
    focal_transition_heuristic(s1a, s1b) = focal_transition_heuristic_grid2d(solver.env, solution, s1a, s1b)

    # Run the search
    vis = CBSGoalVisitorImplicit(env, constraints)
    a_star_epsilon_states = a_star_light_epsilon_shortest_path_implicit!(env.state_graph,
                                                                        edge_wt_fn, idx,
                                                                        vis,
                                                                        solver.weight,
                                                                        admissible_heuristic,
                                                                        focal_state_heuristic,
                                                                        focal_transition_heuristic,
                                                                        Int64)

    plan_result = get_plan_result_from_astar(env, a_star_epsilon_states.dists,
                                            a_star_epsilon_states.parent_indices, idx,
                                            env.curr_goal_idx, a_star_epsilon_states.best_fvalue)

    if plan_result == nothing
        return PlanResult{Grid2DState,Grid2DAction,Int64}()
    end

    return plan_result
end
