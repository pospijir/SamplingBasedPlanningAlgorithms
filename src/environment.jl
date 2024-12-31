using ArgCheck


struct Point
    x::Float32
    y::Float32
end

struct Pose
    x::Float32
    y::Float32
    angle::Float32
end

struct Range
    min::Float32
    max::Float32
end

Base.show(io::IO, p::Point) = print(io, "($(p.x), $(p.y))")
Base.show(io::IO, p::Pose) = print(io, "($(p.x), $(p.y), α=$(p.angle))")
Base.show(io::IO, r::Range) = print(io, "($(r.min), $(r.max))")

function extrainfo(object)
    io = IOBuffer()
    show(io, MIME"text/plain"(), object)
    return String(take!(io))
end

##########  Obstacles  #############################################################

abstract type ObstacleType end
abstract type CircleObstacle <: ObstacleType end
abstract type PolygonObstacle <: ObstacleType end
abstract type RectangleObstacle <: ObstacleType end
abstract type SquareObstacle <: ObstacleType end

struct Obstacle{O <: ObstacleType}
    origin::Point
    radius::Float32
    vertices::NTuple{N, Point} where {N} 
end

function CircleObstacle(origin::Point, radius::Real)
    @argcheck radius > 0 "CircleObstacle radius must be greater than zero: radius=$(radius)"
    Obstacle{CircleObstacle}(origin, radius, ())
end

function PolygonObstacle(vertices::NTuple{N, Point}) where {N}
    @argcheck length(vertices) >= 3 "PolygonObstacle must have at least 3 vertices: vertices=$(vertices)"
    @argcheck length(Set(vertices)) == length(vertices) "PolygonObstacle vertices must be unique: vertices=$(vertices)"
    
    n = length(vertices)
    mean_x = sum(v.x for v in vertices) / n
    mean_y = sum(v.y for v in vertices) / n
    origin = Point(mean_x, mean_y)

    distances_squared = (getfield.(vertices, :x) .- origin.x).^2 .+ (getfield.(vertices, :y) .- origin.y).^2
    radius = sqrt(maximum(distances_squared))

    return Obstacle{PolygonObstacle}(origin, radius, vertices)
end

function RectangleObstacle(origin::Point, a::Real, b::Real, angle::Real=0, obstacle_type::Type{<:ObstacleType}=RectangleObstacle)
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
        return Point(rotated_x, rotated_y)
    end, length(unrotated_vertices))

    radius = sqrt(half_a ^ 2 + half_b ^ 2)
    
    return Obstacle{obstacle_type}(origin, radius, rotated_vertices)
end

SquareObstacle(origin::Point, a::Real, angle::Real=0) = RectangleObstacle(origin, a, a, angle, SquareObstacle)

function Base.show(io::IO, o::Obstacle{O}) where {O <: ObstacleType}
    print(io, "$O: origin=$(o.origin), radius=$(o.radius)")
end

function Base.show(io::IO, ::MIME"text/plain", o::Obstacle{O}) where {O <: ObstacleType}
    print(io, """
    Obstacle:
        - Type: $O
        - Origin: $(o.origin)
        - Radius: $(o.radius)
        - Vertices: $(o.vertices)
    """)
end


##########  Agent  #############################################################

abstract type AgentType end
abstract type PointAgent <: AgentType end
abstract type CircleAgent <: AgentType end
abstract type PolygonAgent <: AgentType end

mutable struct Agent{A <: AgentType}
    origin::Point
    radius::Float32
    vertices::NTuple{N, Point} where {N}
end

PointAgent(origin::Point) = Agent{PointAgent}(origin, 0, ())

function CircleAgent(origin::Point, radius::Real)
    @argcheck radius > 0 "CircleAgent radius must be greater than zero: radius=$(radius)"
    Agent{CircleAgent}(origin, radius, ())
end

function PolygonAgent(origin::Point, vertices::NTuple{N, Point}) where {N}
    @argcheck length(vertices) >= 3 "PolygonAgent must have at least 3 vertices: vertices=$(vertices)"
    @argcheck length(Set(vertices)) == length(vertices) "PolygonAgent vertices must be unique: vertices=$(vertices)"
    
    distances_squared = (getfield.(vertices, :x) .- origin.x).^2 .+ (getfield.(vertices, :y) .- origin.y).^2
    radius = sqrt(maximum(distances_squared))

    return Agent{PolygonAgent}(origin, radius, vertices)
end

function PolygonAgent(vertices::NTuple{N, Point}) where {N}
    @argcheck length(vertices) >= 3 "PolygonAgent must have at least 3 vertices: vertices=$(vertices)"
    @argcheck length(Set(vertices)) == length(vertices) "PolygonAgent vertices must be unique: vertices=$(vertices)"
    
    n = length(vertices)
    mean_x = sum(v.x for v in vertices) / n
    mean_y = sum(v.y for v in vertices) / n
    origin = Point(mean_x, mean_y)

    return PolygonAgent(origin,  vertices)
end

function Base.show(io::IO, a::Agent{A}) where {A <: AgentType}
    print(io, "$A: origin=$(a.origin), radius=$(a.radius)")
end

function Base.show(io::IO, ::MIME"text/plain", a::Agent{A}) where {A <: AgentType}
    print(io, """
    Agent:
        - Type: $A
        - Origin: $(a.origin)
        - Radius: $(a.radius)
        - Vertices: $(a.vertices)
    """)
end

function transform_vertices(agent::Agent{<:AgentType}, pose::Pose)
    cos_angle = cos(pose.angle)
    sin_angle = sin(pose.angle)
    
    return ntuple(i -> begin
        v = agent.vertices[i]
        rotated_x = cos_angle * (v.x - agent.origin.x) - sin_angle * (v.y - agent.origin.y) + agent.origin.x
        rotated_y = sin_angle * (v.x - agent.origin.x) + cos_angle * (v.y - agent.origin.y) + agent.origin.y
        x = rotated_x + (pose.x - agent.origin.x)
        y = rotated_y + (pose.y - agent.origin.y)
        return Point(x, y)
    end, length(agent.vertices))
end


##########  World  #############################################################

struct World
    name::String
    start::Pose
    goal::Pose
    xlim::Range
    ylim::Range
    obstacles::NTuple{N, Obstacle} where {N}
end

function Base.show(io::IO, w::World)
    print(io, "World: $(w.name), start=$(w.start), goal=$(w.goal), #obstacles=$(length(w.obstacles))")
end

function Base.show(io::IO, ::MIME"text/plain", w::World)
    print(io, """
    World: $(w.name)
        - Start: $(w.start)
        - Goal: $(w.goal)
        - Xlim: $(w.xlim)
        - Ylim: $(w.ylim)
        - #Obstacles: $(length(w.obstacles))
    """)
    for o in w.obstacles
        print(io, extrainfo(o))
    end
end