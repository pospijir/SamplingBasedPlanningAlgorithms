@testset "Agent Constructors ArgCheck" begin
    @test CircleAgent(Point(1, 2), 1) isa Any
    @test_throws ArgumentError CircleAgent(Point(1, 2), 0)
    @test_throws ArgumentError CircleAgent(Point(1, 2), -1f0)
    @test_throws ArgumentError CircleAgent(Point(1, 2), -2)

    @test PolygonAgent((Point(1, 2), Point(1, -2), Point(-1, 2), Point(3, 2))) isa Any
    @test_throws ArgumentError PolygonAgent((Point(1, 2), Point(1, -2), Point(1, 2), Point(3, 2)))
    @test_throws ArgumentError PolygonAgent((Point(1, -2), Point(-1, 2)))
end