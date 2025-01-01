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

    @test isapprox(distance_squared(Point(2, 0), Point(1, 0), CircleObstacle(Point(-1, -1), 1)), 5)
    @test isapprox(distance_squared(Point(1, 0), Point(2, 0), CircleObstacle(Point(-1, -1), 1)), 5)
    @test isapprox(distance_squared(Point(-1, 1), Point(2, 1), CircleObstacle(Point(0, 0), 1)), 1)
    @test isapprox(distance_squared(Point(2, -1), Point(2, 1), CircleObstacle(Point(0, 0), 1)), 4)
end

@testset "Point Inside Polygon" begin
    square = SquareObstacle(Point(0, 0), 2)
    @test is_inside(Point(0, 0), square.vertices) == true
    @test is_inside(Point(2, 2), square.vertices) == false
    for i in 1:length(square.vertices)
        @test is_inside(square.vertices[1], square.vertices) == true
    end

    for _ in 1:25
        x = rand() * 4 - 2
        y = rand() * 4 - 2
        @test is_inside(Point(x, y), square.vertices) == (abs(x) <= 1 && abs(y) <= 1)
    end
end

@testset "Collisions" begin
    testset_agent_name = "PointAgent"
    @testset "$testset_agent_name" begin
        testset_obstacle_name = "CircleObstacle"
        @testset "$testset_obstacle_name" begin

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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-1.png"), fig)
            
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-2.png"), fig)

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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-3.png"), fig)

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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-4.png"), fig)

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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-5.png"), fig)

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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-6.png"), fig)

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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-7.png"), fig)
        end

        testset_obstacle_name = "PolygonObstacle"
        @testset "$testset_obstacle_name" begin
            agent = PointAgent(Point(0, 0))
            pose = Pose(-sqrt(0.5), sqrt(0.5), 0)
            obstacle = SquareObstacle(Point(0, 0), 2)
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-1.png"), fig)

            agent = PointAgent(Point(0, 0))
            pose = Pose(-1, sqrt(0.5), 0)
            obstacle = SquareObstacle(Point(0, 0), 2)
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-2.png"), fig)

            agent = PointAgent(Point(0, 0))
            pose = Pose(-1.2, 0, 0)
            obstacle = SquareObstacle(Point(0, 0), 2)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-2, 2),
                Range(-2, 2),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == true
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-3.png"), fig)

            agent = PointAgent(Point(0, 0))
            pose = Pose(0, 0, 0)
            obstacle = PolygonObstacle((
                Point(-1, 1), Point(-0.5, 1), Point(-0.5, -0.5), Point(0.5, -0.5), 
                Point(0.5, 1), Point(1, 1), Point(1, -1), Point(-1, -1)
            ))
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-2, 2),
                Range(-2, 2),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == true
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-4.png"), fig)

            agent = PointAgent(Point(0, 0))
            pose = Pose(-0.7, 0, 0)
            obstacle = PolygonObstacle((
                Point(-1, 1), Point(-0.5, 1), Point(-0.5, -0.5), Point(0.5, -0.5), 
                Point(0.5, 1), Point(1, 1), Point(1, -1), Point(-1, -1)
            ))
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-5.png"), fig)
        end
    end


    testset_agent_name = "CircleAgent"
    @testset "$testset_agent_name" begin
        testset_obstacle_name = "CircleObstacle"
        @testset "$testset_obstacle_name" begin

            agent = CircleAgent(Point(0, 0), 1)
            pose = Pose(0, 0, 0)
            obstacle = CircleObstacle(Point(2, 0), 1)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-2, 4),
                Range(-2, 2),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == false
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-1.png"), fig)
            
            agent = CircleAgent(Point(0, 0), 1)
            pose = Pose(0, 0, 0)
            obstacle = CircleObstacle(Point(1, 1), 1)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-2, 3),
                Range(-2, 3),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == false
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-2.png"), fig)

            agent = CircleAgent(Point(0, 0), 0.4)
            pose = Pose(0, 1, 0)
            obstacle = CircleObstacle(Point(0, 2), 0.5)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-3, 3),
                Range(-1, 5),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == true
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-3.png"), fig)

            agent = CircleAgent(Point(0, 0), 0.5)
            pose = Pose(-sqrt(0.5), sqrt(0.5), 0)
            obstacle = CircleObstacle(Point(0, 0), 0.2)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-2, 2),
                Range(-2, 2),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == true
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-4.png"), fig)
        end
    end
end