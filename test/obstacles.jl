@testset "Obstacle Constructors ArgCheck" begin
    @test CircleObstacle(1, Point(1, 2), 1) isa Any
    @test_throws ArgumentError CircleObstacle(1, Point(1, 2), 0)
    @test_throws ArgumentError CircleObstacle(1, Point(1, 2), -1f0)
    @test_throws ArgumentError CircleObstacle(1, Point(1, 2), -2)

    @test PolygonObstacle(1, (Point(1, 2), Point(1, -2), Point(-1, 2), Point(3, 2))) isa Any
    @test_throws ArgumentError PolygonObstacle(1, (Point(1, 2), Point(1, -2), Point(1, 2), Point(3, 2)))
    @test_throws ArgumentError PolygonObstacle(1, (Point(1, -2), Point(-1, 2)))

    @test SquareObstacle(1, Point(1, 2), 2, 30) isa Any
    @test_throws ArgumentError SquareObstacle(1, Point(1, 2), 0, 30)
    @test_throws ArgumentError SquareObstacle(1, Point(1, 2), -1f0, 30)
    @test_throws ArgumentError SquareObstacle(1, Point(1, 2), -2, 30)
  
    @test RectangleObstacle(1, Point(1, 2), 2, 1, 30) isa Any
    @test_throws ArgumentError RectangleObstacle(1, Point(1, 2), 0, 1, 30)
    @test_throws ArgumentError RectangleObstacle(1, Point(1, 2), 1, -1f0, 30)
    @test_throws ArgumentError RectangleObstacle(1, Point(1, 2), -2, 2, 30)
end


function compare_obstacles(o1::Obstacle{<:ObstacleType}, o2::Obstacle{<:ObstacleType}; atol=1e-5)
    @test isapprox(o1.radius, o2.radius; atol=atol)

    @test isapprox(o1.origin.x, o2.origin.x; atol=atol)
    @test isapprox(o1.origin.y, o2.origin.y; atol=atol)

    @test length(o1.vertices) == length(o2.vertices)
    for (v1, v2) in zip(o1.vertices, o2.vertices)
        @test isapprox(v1.x, v2.x; atol=atol)
        @test isapprox(v1.y, v2.y; atol=atol)
    end
end


@testset "Obstacle Constructors" begin
    
    sqrt2 = sqrt(2)
    
    origin1 = Point(0, 0)
    radius1 = sqrt2
    vertices1 = (Point(-1, -1), Point(1, -1), Point(1, 1), Point(-1, 1))
    a1 = 2
    angle1 = 0
    obs1 = Obstacle{ObstacleType}(0, origin1, radius1, vertices1)
    sq_obs1 = SquareObstacle(0, origin1, a1, angle1)
    pol_obs1 = PolygonObstacle(0, vertices1)
    
    compare_obstacles(obs1, sq_obs1)
    compare_obstacles(obs1, pol_obs1)
    compare_obstacles(sq_obs1, pol_obs1)

    origin2 = Point(0, 0)
    radius2 = sqrt2
    vertices2 = (Point(1, -1), Point(1, 1), Point(-1, 1), Point(-1, -1))
    a2 = 2
    angle2 = 90
    obs2 = Obstacle{ObstacleType}(0, origin2, radius2, vertices2)
    sq_obs2 = SquareObstacle(0, origin2, a2, angle2)
    
    compare_obstacles(obs2, sq_obs2)

    origin3 = Point(0, 0)
    radius3 = sqrt2
    vertices3 = (Point(0, -sqrt2), Point(sqrt2, 0), Point(0, sqrt2), Point(-sqrt2, 0))
    a3 = 2
    angle3 = 45
    obs3 = Obstacle{ObstacleType}(0, origin3, radius3, vertices3)
    sq_obs3 = SquareObstacle(0, origin3, a3, angle3)
    
    compare_obstacles(obs3, sq_obs3)

    origin4 = Point(5, -6)
    vertices4 = (Point(0, -8.5), Point(10, -8.5), Point(10, -3.5), Point(0, -3.5))
    a4 = 10
    b4 = 5
    angle4 = 0
    rec_obs4 = RectangleObstacle(0, origin4, a4, b4, angle4)
    pol_obs4 = PolygonObstacle(0, vertices4)
    
    compare_obstacles(rec_obs4, pol_obs4)
end