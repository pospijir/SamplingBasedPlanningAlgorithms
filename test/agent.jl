@testset "Agent Constructors ArgCheck" begin
    @test CircleAgent(Point(1, 2), 1) isa Any
    @test_throws ArgumentError CircleAgent(Point(1, 2), 0)
    @test_throws ArgumentError CircleAgent(Point(1, 2), -1f0)
    @test_throws ArgumentError CircleAgent(Point(1, 2), -2)

    @test PolygonAgent((Point(1, 2), Point(1, -2), Point(-1, 2), Point(3, 2))) isa Any
    @test_throws ArgumentError PolygonAgent((Point(1, 2), Point(1, -2), Point(1, 2), Point(3, 2)))
    @test_throws ArgumentError PolygonAgent((Point(1, -2), Point(-1, 2)))
end

@testset "Distance Squared" begin
    @test isapprox(distance_squared(Point(0, 0), CircleObstacle(Point(0, 0), 1)), 0)
    @test isapprox(distance_squared(Point(2, 0), CircleObstacle(Point(0, 0), 1)), 4)
    @test isapprox(distance_squared(Point(0, 0), CircleObstacle(Point(1, 1), 1)), 2)
    @test isapprox(distance_squared(Point(1, 1), CircleObstacle(Point(4, 5), 1)), 25)
    @test isapprox(distance_squared(Point(-1, -1), CircleObstacle(Point(-5, -4), 1)), 25)
    @test isapprox(distance_squared(Point(-1, -1), CircleObstacle(Point(1, 2), 1)), 13)
end

@testset "Collisions" begin
    @testset "PointAgent" begin
        agent = PointAgent(Point(0, 0))
        pose = Pose(0, 0, 0)
        obstacle = CircleObstacle(Point(2, 0), 1)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-1, 4),
            Range(-2, 2),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == true
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-1.png"), fig)
        
        agent = PointAgent(Point(0, 0))
        pose = Pose(0, 0, 0)
        obstacle = CircleObstacle(Point(1, 1), 1)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-1, 3),
            Range(-1, 3),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == true
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-2.png"), fig)

        agent = PointAgent(Point(0, 0))
        pose = Pose(0, 0, 0)
        obstacle = CircleObstacle(Point(2, 0), 2)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-1, 5),
            Range(-3, 3),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == false
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-3.png"), fig)

        agent = PointAgent(Point(0, 0))
        pose = Pose(1, 0, 0)
        obstacle = CircleObstacle(Point(2, 0), 2)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-1, 5),
            Range(-3, 3),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == false
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-4.png"), fig)

        agent = PointAgent(Point(0, 0))
        pose = Pose(-0.5, 0.5, 0)
        obstacle = CircleObstacle(Point(0, 0), 1)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-2, 2),
            Range(-2, 2),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == false
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-5.png"), fig)

        agent = PointAgent(Point(0, 0))
        pose = Pose(0, 1, 0)
        obstacle = CircleObstacle(Point(0, 2), 2)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-3, 3),
            Range(-1, 5),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == false
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-6.png"), fig)

        agent = PointAgent(Point(0, 0))
        pose = Pose(-sqrt(0.5), sqrt(0.5), 0)
        obstacle = CircleObstacle(Point(0, 0), 1)
        world = World(
            "Test Collisions",
            pose,
            Pose(100, 0, 0),
            Range(-2, 2),
            Range(-2, 2),
            (obstacle,)
        )
        @test is_collision_free(agent, pose, obstacle) == false
        fig = plot_environment(agent, world)
        save(joinpath("plots", "collisions-pointagent-7.png"), fig)
    end
end