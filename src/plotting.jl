using CairoMakie: poly!, xlims!, ylims!, Figure, Axis, Label
using Makie.GeometryBasics: Circle, Point2f


const AGENT_COLOR = :blue
const AGENT_BORDER_COLOR = :black
const AGENT_BORDER_WIDTH = 2
const AGENT_POINT_RADIUS = 0.1
const OBSTACLE_COLOR = :black

function plot_agent!(a::Agent{CircleAgent}, color::Symbol=AGENT_COLOR)
    poly!(Circle(
        Point2f(a.origin.x, a.origin.y), a.radius),
        color=color,
        strokecolor=AGENT_BORDER_COLOR,
        strokewidth=AGENT_BORDER_WIDTH
    )
end

function plot_agent!(a::Agent{PointAgent}, color::Symbol=AGENT_COLOR)
    poly!(Circle(
        Point2f(a.origin.x, a.origin.y), AGENT_POINT_RADIUS),
        color=color,
        strokecolor=AGENT_BORDER_COLOR,
        strokewidth=AGENT_BORDER_WIDTH
    )
end

function plot_agent!(a::Agent{PolygonAgent}, color::Symbol=AGENT_COLOR)
    points = Point2f[[(vertex.x, vertex.y) for vertex in a.vertices]...]
    poly!(
        points,
        color=color,
        strokecolor=AGENT_BORDER_COLOR,
        strokewidth=AGENT_BORDER_WIDTH
    )
end

function plot_agent!(a::Agent{CircleAgent}, pose::Pose, color::Symbol=AGENT_COLOR)
    poly!(Circle(
        Point2f(a.origin.x + pose.x, a.origin.y + pose.y), a.radius),
        color=color,
        strokecolor=AGENT_BORDER_COLOR,
        strokewidth=AGENT_BORDER_WIDTH
    )
end

function plot_agent!(a::Agent{PointAgent}, pose::Pose, color::Symbol=AGENT_COLOR)
    poly!(Circle(
        Point2f(a.origin.x + pose.x, a.origin.y + pose.y), AGENT_POINT_RADIUS),
        color=color,
        strokecolor=AGENT_BORDER_COLOR,
        strokewidth=AGENT_BORDER_WIDTH
    )
end

function plot_agent!(a::Agent{PolygonAgent}, pose::Pose, color::Symbol=AGENT_COLOR)
    vertices = transform_vertices(a, pose)
    points = Point2f[[(vertex.x, vertex.y) for vertex in vertices]...]
    poly!(
        points,
        color=color,
        strokecolor=AGENT_BORDER_COLOR,
        strokewidth=AGENT_BORDER_WIDTH
    )
end

function plot_obstacle!(o::Obstacle{CircleObstacle})
    poly!(Circle(Point2f(o.origin.x, o.origin.y), o.radius), color=OBSTACLE_COLOR)
end

function plot_obstacle!(o::Obstacle{O}) where {O <: ObstacleType}
    points = Point2f[[(vertex.x, vertex.y) for vertex in o.vertices]...]
    poly!(points, color=OBSTACLE_COLOR)
end

function plot_world!(w::World, fig::Figure)
    ax = Axis(fig[1, 1], title=w.name)
    ax.ylabel = "y"
    ax.xlabel = "x"
    xlims!(ax, w.xlim.min, w.xlim.max)
    ylims!(ax, w.ylim.min, w.ylim.max)

    for o in w.obstacles
        plot_obstacle!(o)
    end

end

function plot_environment(a::Agent{A}, w::World, size::Int=640) where {A <: AgentType}
    fig = Figure(size=(size, size))
    plot_world!(w, fig)
    plot_agent!(a, w.start)
    plot_agent!(a, w.goal, :green)

    return fig
end