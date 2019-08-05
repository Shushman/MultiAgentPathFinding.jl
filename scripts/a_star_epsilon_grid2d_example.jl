import JSON
using MultiAgentPathFinding

function main(infile::String, start_x::Int64, start_y::Int64, goal_x::Int64, goal_y::Int64, weight::Float64)

    config = JSON.parsefile(infile)

    dim = convert(Vector{Int64}, config["map"]["dimensions"])

    obstacles = Set{Grid2DLocation}()
    for obs in config["map"]["obstacles"]
        obs_arr = convert(Vector{Int64}, obs)
        push!(obstacles, Grid2DLocation((x=obs_arr[1], y=obs_arr[2])))
    end

    start = Grid2DLocation((x=start_x, y=start_y))
    goal = Grid2DLocation((x=goal_x, y=goal_y))

    env = AStarGrid2DEnvironment(dimx=dim[1], dimy=dim[2], goal=goal, obstacles=obstacles)

    @time solution, cost = low_level_search!(env, start, weight)

    if solution == nothing
        println("Plannign not successful!")
    else
        println("Solution: ")
        for s in solution
            println(s)
        end

        println("Cost: ",cost)
    end
end
