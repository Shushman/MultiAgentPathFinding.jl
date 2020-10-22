@with_kw struct Grid2DCGState
    grid2d_state::Grid2DState
    coord_mode::Int64 = 0
end

@with_kw struct Grid2DCGAction <: MAPFAction
    action::Action
    coord_mode::Int64 = 0   # Type of coordination (simplest is 0/1)
end
