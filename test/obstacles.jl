@testset "Obstacle Constructors" begin
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