@with_kw struct Grid2DState <: MAPFState
    time::Int64
    x::Int64
    y::Int64
end
Base.isequal(s1::Grid2DState, s2::Grid2DState) = (s1.time, s1.x, s1.y) == (s2.time, s2.x, s2.y)

function equal_except_time(s1::Grid2DState, s2::Grid2DState)
    return (s1.x == s2.x) && (s1.y == s2.y)
end

@enum Action Up=1 Down=2 Left=3 Right=4 Wait=5

struct Grid2DAction <: MAPFAction
    action::Action
end

@enum ConflictType Vertex=1 Edge=2

@with_kw struct Grid2DConflict <: MAPFConflict
    time::Int64
    agent_1::Int64
    agent_2::Int64
    type::ConflictType
    x1::Int64       = -1
    y1::Int64       = -1
    x2::Int64       = -1
    y2::Int64       = -1
end


@with_kw struct VertexConstraint
    time::Int64
    x::Int64
    y::Int64
end

Base.isless(vc1::VertexConstraint, vc2::VertexConstraint) = (vc1.time, vc1.x, vc1.x) < (vc2.time, vc2.x, vc2.y)
Base.isequal(vc1::VertexConstraint, vc2::VertexConstraint) = (vc1.time, vc1.x, vc1.y) == (vc2.time, vc2.x, vc2.y)

@with_kw struct EdgeConstraint
    time::Int64
    x1::Int64
    y1::Int64
    x2::Int64
    y2::Int64
end

Base.isless(ec1::EdgeConstraint, ec2::EdgeConstraint) = (ec1.time, ec1.x1, ec1.y1, ec1.x2, ec1.y2) < (ec2.time, ec2.x1, ec2.y1, ec2.x2, ec2.y2)
Base.isequal(ec1::EdgeConstraint, ec2::EdgeConstraint) = (ec1.time, ec1.x1, ec1.y1, ec1.x2, ec1.y2) == (ec2.time, ec2.x1, ec2.y1, ec2.x2, ec2.y2)

@with_kw struct Grid2DConstraints <: MAPFConstraints
    vertex_constraints::Set{VertexConstraint}   = Set{VertexConstraint}()
    edge_constraints::Set{EdgeConstraint}       = Set{EdgeConstraint}()
end

get_empty_constraint(Grid2DConstraints) = Grid2DConstraints()

Base.isempty(constraints::Grid2DConstraints) = (isempty(constraints.vertex_constraints) && isempty(constraints.edge_constraints))

const Grid2DLocation = NamedTuple{(:x,:y),Tuple{Int64,Int64}}
