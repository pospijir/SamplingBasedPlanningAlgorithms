using CairoMakie
using Makie.GeometryBasics


AGENT_COLOR = :blue
AGENT_BORDER_COLOR = :black
AGENT_BORDER_WIDTH = 2

OBSTACLE_COLOR = :black

function plot!(a::Agent{CircleAgent})
    poly!(Circle(Point2f(a.pose.x, a.pose.y), a.radius), color=AGENT_COLOR, strokecolor=AGENT_BORDER_COLOR, strokewidth=AGENT_BORDER_WIDTH)
end

function plot!(a::Agent{PointAgent}, radius=0.1)
    poly!(Circle(Point2f(a.pose.x, a.pose.y), radius), color=AGENT_COLOR, strokecolor=AGENT_BORDER_COLOR, strokewidth=AGENT_BORDER_WIDTH)
end

function plot!(a::Agent{PolygonAgent})
    points = Point2f[[(vertex.x, vertex.y) for vertex in a.vertices]...]
    poly!(points, color=OBSTACLE_COLOR)
end

function plot!(o::Obstacle{CircleObstacle})
    poly!(Circle(Point2f(o.pose.x, o.pose.y), o.radius), color=OBSTACLE_COLOR)
end

function plot!(o::Obstacle{O}) where {O <: ObstacleType}
    points = Point2f[[(vertex.x, vertex.y) for vertex in o.vertices]...]
    poly!(points, color=OBSTACLE_COLOR)
end

function plot!(w::World)
    for o in w.obstacles
        plot!(o)
    end
end