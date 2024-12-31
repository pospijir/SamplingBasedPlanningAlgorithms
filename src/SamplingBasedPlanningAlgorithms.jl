module SamplingBasedPlanningAlgorithms

include("environment.jl")
export Point, Pose, Range
export Obstacle, CircleObstacle, PolygonObstacle, RectangleObstacle, SquareObstacle, ObstacleType
export PointAgent, CircleAgent, PolygonAgent
export World
export extrainfo
export distance_squared, is_collision_free 

include("plotting.jl")
export plot_agent!, plot_obstacle!, plot_world!, plot_environment

end # module SamplingBasedPlanningAlgorithms
