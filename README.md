# MultiAgentPathFinding.jl

A Julia implementation of two fundamental Multi-Agent Path Finding (MAPF) algorithms -
[Conflict-Based Search](https://www.sciencedirect.com/science/article/pii/S0004370214001386) or CBS,
and its bounded-suboptimal variant, [Enhanced CBS](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/view/8911).
This repository is heavily based on a [C++ library](https://github.com/whoenig/libMultiRobotPlanning) for multi-robot planning
(with comparable performance - see below).

## Note for potential users
The primary purpose of this repository is as an efficient implementation of Enhanced CBS
for my paper _Enhanced Multi-Drone Delivery Using Transit Networks_ ([ArXiv](), [Code](https://github.com/sisl/MultiAgentAllocationTransit.jl/tree/master/src)).
**I have no immediate plans to implement other MAPF algorithms here.**
Additionally, the low-level search implementation in the `domains/`
folder that I have provided depend on my fork of the (archived)
[Graphs.jl](https://github.com/Shushman/Graphs.jl) package. My fork has implementations
for A* and Focal Search and weight-constrained versions of both with an _implicit
graph structure_ where edges are not explicitly enumerated and out-neighbors
are computed just-in-time using a visitor function (which [LightGraphs.jl](https://github.com/JuliaGraphs/LightGraphs.jl) does
not support, at least when I [last checked](https://github.com/JuliaGraphs/LightGraphs.jl/issues/1108)). </br>
That said, the templated code for the abstract types and (E)CBS are very light on dependencies and easy-to-extend to other MAPF algorithms; feel free to fork and extend, and I'm happy to help to some extent if I can.


## If you do want to use this...
The MultiAgentPathFinding repository is set up as a package with its own environment in [Julia 1.0](https://julialang.org/downloads/). Look at **Using someone else's project** at the Julia [package manager documentation](https://julialang.github.io/Pkg.jl/v1/environments/#Using-someone-else's-project-1) for the basic idea. To get the code up and running (after having installed Julia), first `cd` into the `MultiAgentPathFinding` folder.
Then start the Julia REPL and go into [package manager](https://julialang.github.io/Pkg.jl/v1/getting-started/) mode by pressing `]`, followed by:
```shell
(v1.0) pkg> activate .
(MultiAgentPathFinding) pkg> instantiate
```
This will install the necessary dependencies and essentially reproduce the Julia environment required to make the package work. You can test this by exiting the package manager mode with the backspace key and then in the Julia REPL entering:
```shell
julia> using MultiAgentPathFinding
```
The full package should then pre-compile.

### Running an Example
The `scripts/` folder has an example for each of CBS and ECBS on the 2D Grid World domain (a standard benchmark for MAPF algorithms - see the ECBS paper).
The `main` function in each script has an `infile::String` argument.
The infiles here refer to the `benchmark` files in the reference C++ repository - see [here](https://github.com/whoenig/libMultiRobotPlanning/tree/master/benchmark).
**Please Note**, you need to use JSON versions of the YAML files (I had some trouble getting the YAML files to play nicely). I have a simple converter `scripts/yaml_to_json.py` that you can run on the downloaded benchmark folder for that purpose.

Once you've done all that, running an example is pretty easy. Assuming you have a `benchmarks/` folder at the top-level `benchmarks/8x8_obst12/map_8x8_*.yaml` file (from the C++ reference), you can do the following (while having the environment activated):
```shell
julia> include("scripts/cbs_grid2d_example.jl")
julia> main("./benchmarks/<your-env-filename>.yaml", 1.5, "<some-out-file>.json", "SOC")
```
where 1.5 is the w sub-optimality factor for focal search, and `"SOC"` refers to the sum-of-costs high-level objective (could also be `"MS"` for makespan).
The call to `main` also outputs the time required for (E)CBS through the `@time` macros. You have to discard the timing from the first call to `main` as it triggers compilation and the [timing is higher than the true one](https://docs.julialang.org/en/v1/manual/performance-tips/index.html#Measure-performance-with-[@time](@ref)-and-pay-attention-to-memory-allocation-1).
You can visualize the output solution file by using `scripts/visualize.py` (which has been [copied over](https://github.com/whoenig/libMultiRobotPlanning/blob/master/example/visualize.py) from the C++ reference repository).


### A quick note on performance
Take this with a grain of salt as I have not tried to optimize my implementation completely (nor, certainly, have the C++ repository authors). However, the Julia implementation appears to have comparable computation time as compared to the C++ one
(the RAM usage is higher for the Grid 2D example, though I did not really try to streamline the domain implementation).
For what it's worth, here are the numbers for both implementations on my machine, for the exact same problem and algorithm (and the exact same result), with one example each of CBS and ECBS:
