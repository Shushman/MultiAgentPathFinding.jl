import JSON
using MultiAgentPathFinding

function main(infile::String, outfile::String, hlcost::String)

    # Main code block
    config = JSON.parsefile(infile)

    dim = convert(Vector{Int64}, config["map"]["dimensions"])

    obstacles = Set{Grid2DLocation}()
    for obs in config["map"]["obstacles"]
        obs_arr = convert(Vector{Int64}, obs)
        push!(obstacles, Grid2DLocation((x=obs_arr[1], y=obs_arr[2])))
    end

    initial_states = Vector{Grid2DState}(undef, 0)
    goals = Vector{Grid2DLocation}(undef, 0)

    for agent in config["agents"]

        agent_loc = convert(Vector{Int64}, agent["start"])
        push!(initial_states, Grid2DState(time=0, x=agent_loc[1], y=agent_loc[2]))

        goal_loc = convert(Vector{Int64}, agent["goal"])
        push!(goals, Grid2DLocation((x=goal_loc[1], y=goal_loc[2])))

    end

    env = Grid2DEnvironment(dimx=dim[1], dimy=dim[2], goals=goals, obstacles=obstacles)

    ## TODO: Definitely a better way to do this....
    if hlcost == "SOC"
        solver = CBSSolver{Grid2DState,Grid2DAction,Int64,SumOfCosts,Grid2DConflict,Grid2DConstraints,Grid2DEnvironment}(env=env)
    elseif hlcost == "MS"
        solver = CBSSolver{Grid2DState,Grid2DAction,Int64,Makespan,Grid2DConflict,Grid2DConstraints,Grid2DEnvironment}(env=env)
    else
        println("Please enter either SOC or MS for high level cost")
    end

    @time solution = search!(solver, initial_states)

    if isempty(solution)
        println("Planning not successful!")
    else
        cost = 0
        makespan = 0
        for s in solution
            cost += s.cost
            makespan = max(s.cost, makespan)
        end

        println("Statistics :")
        println("Cost: ", cost)
        println("Makespan: ", makespan)

        stats_dict = Dict("cost"=>cost, "makespan"=>makespan)

        schedule_dict = Dict()

        for (a, soln) in enumerate(solution)
            soln_dict = Vector{Dict}(undef,0)
            for (state, t) in soln.states
                push!(soln_dict, Dict("x"=>state.x, "y"=>state.y, "t"=>t))
            end

            schedule_dict[string("agent",a-1)] = soln_dict
        end
    end

    result_dict = Dict("statistics"=>stats_dict, "schedule"=>schedule_dict)

    open(outfile, "w") do f
        JSON.print(f, result_dict, 2)
    end
end
