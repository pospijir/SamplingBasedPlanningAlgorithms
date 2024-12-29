@testset "Agent Constructors ArgCheck" begin
    @test CircleAgent(1, Pose(1, 2, 0), 1) isa Any
    @test_throws ArgumentError CircleAgent(1, Pose(1, 2, 0), 0)
    @test_throws ArgumentError CircleAgent(1, Pose(1, 2, 0), -1f0)
    @test_throws ArgumentError CircleAgent(1, Pose(1, 2, 0), -2)

    @test PolygonAgent(1, (Point(1, 2), Point(1, -2), Point(-1, 2), Point(3, 2))) isa Any
    @test_throws ArgumentError PolygonAgent(1, (Point(1, 2), Point(1, -2), Point(1, 2), Point(3, 2)))
    @test_throws ArgumentError PolygonAgent(1, (Point(1, -2), Point(-1, 2)))
end