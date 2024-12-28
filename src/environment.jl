using ArgCheck


struct Point
    x::Float32
    y::Float32
end


##########  Obstacles  #############################################################

abstract type ObstacleTypes end
abstract type CircleObstacle <: ObstacleTypes end
abstract type PolygonObstacle <: ObstacleTypes end
abstract type RectangleObstacle <: ObstacleTypes end
abstract type SquareObstacle <: ObstacleTypes end

struct Obstacle{O <: ObstacleTypes}
    id::Int
    origin::Point
    radius::Float32
    vertices::NTuple{N, Point} where {N} 
end

function CircleObstacle(id::Int, origin::Point, radius::Real)
    @argcheck radius > 0 "CircleObstacle radius must be greater than zero: radius=$(radius)"
    Obstacle{CircleObstacle}(id, origin, radius, ())
end

function PolygonObstacle(id::Int, vertices::NTuple{N, Point}) where {N}

    @argcheck length(vertices) >= 3 "PolygonObstacle must have at least 3 vertices: vertices=$(vertices)"
    @argcheck length(Set(vertices)) == length(vertices) "PolygonObstacle vertices must be unique: vertices=$(vertices)"
    
    n = length(vertices)
    mean_x = sum(v.x for v in vertices) / n
    mean_y = sum(v.y for v in vertices) / n
    origin = Point(mean_x, mean_y)

    distances_squared = (getfield.(vertices, :x) .- origin.x).^2 .+ (getfield.(vertices, :y) .- origin.y).^2
    radius = sqrt(maximum(distances_squared))

    return Obstacle{PolygonObstacle}(id, origin, radius, vertices)
end

function RectangleObstacle(id::Int, origin::Point, a::Real, b::Real, angle::Real, obstacle_type::Type{<:ObstacleTypes}=RectangleObstacle)
    @argcheck a > 0 "RectangleObstacle a must be greater than zero: a=$(a)"
    @argcheck b > 0 "RectangleObstacle b must be greater than zero: b=$(b)"
    
    half_a, half_b = a / 2, b / 2
    
    unrotated_vertices = (
        Point(origin.x - half_a, origin.y - half_b),
        Point(origin.x + half_a, origin.y - half_b),
        Point(origin.x + half_a, origin.y + half_b),
        Point(origin.x - half_a, origin.y + half_b)
    )
    
    angle = deg2rad(angle)
    cos_angle = cos(angle)
    sin_angle = sin(angle)
    
    rotated_vertices = ntuple(i -> begin
        v = unrotated_vertices[i]
        rotated_x = cos_angle * (v.x - origin.x) - sin_angle * (v.y - origin.y) + origin.x
        rotated_y = sin_angle * (v.x - origin.x) + cos_angle * (v.y - origin.y) + origin.y
        Point(rotated_x, rotated_y)
    end, length(unrotated_vertices))

    radius = max(half_a, half_b)
    
    return Obstacle{obstacle_type}(id, origin, radius, rotated_vertices)
end

SquareObstacle(id::Int, origin::Point, a::Real, angle::Real) = RectangleObstacle(id, origin, a, a, angle, SquareObstacle)

Base.show(io::IO, p::Point) = print(io, "($(p.x), $(p.y))")

function Base.show(io::IO, o::Obstacle{O}) where {O <: ObstacleTypes}
    print(io, "$O #$(o.id), origin=$(o.origin), radius=$(o.radius)")
end

function Base.show(io::IO, ::MIME"text/plain", o::Obstacle{O}) where {O <: ObstacleTypes}
    print(io, """
    Obstacle #$(o.id):
        - Type: $O
        - Origin: $(o.origin)
        - Radius: $(o.radius)
        - Vertices: $(o.vertices)
    """)
end

function extrainfo(o::Obstacle{O}) where {O <: ObstacleTypes}
    io = IOBuffer()
    show(io, MIME"text/plain"(), o)
    return String(take!(io))
end
