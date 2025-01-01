using ArgCheck

const ROUNDING_DIGITS = 3

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
abstract type RectangleObstacle <: PolygonObstacle end
abstract type SquareObstacle <: PolygonObstacle end

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

function distance_squared(point_a::Union{Point, Pose}, point_b::Union{Point, Pose})
    return round((point_a.x - point_b.x) ^ 2 + (point_a.y - point_b.y) ^ 2, digits=ROUNDING_DIGITS)
end

distance_squared(point::Union{Point, Pose}, obstacle::Obstacle{<:ObstacleType}) = distance_squared(point, obstacle.origin)

function distance_squared(point::Union{Point, Pose}, point_a::Point, point_b::Point)
    edge_length_squared = distance_squared(point_a, point_b)

    t = (point.x - point_a.x) * (point_b.x - point_a.x) 
    t += (point.y - point_a.y) * (point_b.y - point_a.y)
    t = clamp(t / edge_length_squared, 0.0, 1.0)
    point_closest = Point(point_a.x + t * (point_b.x - point_a.x), point_a.y + t * (point_b.y - point_a.y))
    return distance_squared(point, point_closest)
end

function is_inside(point::Union{Point, Pose}, vertices::NTuple{N, Point}) where {N}
    inside = false
    n = length(vertices)
    index_a = n
    for index_b in 1:n
        if (vertices[index_a].y > point.y) != (vertices[index_b].y > point.y) && 
           (point.x < (vertices[index_b].x - vertices[index_a].x) * (point.y - vertices[index_a].y) / (vertices[index_b].y - vertices[index_a].y) + vertices[index_a].x)
            inside = !inside
        end
        index_a = index_b
    end
    return inside
end

function is_collision_free(agent::Agent{PointAgent}, pose::Pose, obstacle::Obstacle{CircleObstacle})
   return distance_squared(pose, obstacle) > obstacle.radius ^ 2  
end

function is_collision_free(agent::Agent{PointAgent}, pose::Pose, obstacle::Obstacle{<:PolygonObstacle})
    return !is_inside(pose, obstacle.vertices) 
end

function is_collision_free(agent::Agent{CircleAgent}, pose::Pose, obstacle::Obstacle{CircleObstacle})
    return distance_squared(pose, obstacle) > (agent.radius + obstacle.radius) ^ 2  
end

function is_collision_free(agent::Agent{CircleAgent}, pose::Pose, obstacle::Obstacle{<:PolygonObstacle})
    is_inside(pose, obstacle.vertices) && return false
    
    radius_squared = agent.radius ^ 2
    for vertex in obstacle.vertices
        (distance_squared(pose, vertex) <= radius_squared) && return false
    end
    
    n = length(obstacle.vertices)
    index_a = n
    for index_b in 1:n
        (distance_squared(pose, obstacle.vertices[index_a], obstacle.vertices[index_b]) <= radius_squared) && return false
        index_a = index_b
    end

    return true
end

function is_collision_free(agent::Agent{PolygonAgent}, pose::Pose, obstacle::Obstacle{CircleObstacle})
    vertices = transform_vertices(agent, pose)

    is_inside(obstacle.origin, vertices) && return false
    
    radius_squared = obstacle.radius ^ 2
    for vertex in vertices
        (distance_squared(obstacle.origin, vertex) <= radius_squared) && return false
    end
    
    n = length(vertices)
    index_a = n
    for index_b in 1:n
        (distance_squared(obstacle.origin, vertices[index_a], vertices[index_b]) <= radius_squared) && return false
        index_a = index_b
    end

    return true
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