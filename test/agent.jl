using SamplingBasedPlanningAlgorithms: is_inside, distance_squared


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

    @test isapprox(distance_squared(Point(-1, -1), Point(2, 0), Point(1, 0)), 5)
    @test isapprox(distance_squared(Point(-1, -1), Point(1, 0), Point(2, 0)), 5)
    @test isapprox(distance_squared(Point(0, 0), Point(-1, 1), Point(2, 1)), 1)
    @test isapprox(distance_squared(Point(0, 0), Point(2, -1), Point(2, 1)), 4)
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

        testset_obstacle_name = "PolygonObstacle"
        @testset "$testset_obstacle_name" begin
            agent = CircleAgent(Point(0, 0), 0.5)
            pose = Pose(-1, 1, 0)
            obstacle = SquareObstacle(Point(2, 0), 1, 30)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-2, 4),
                Range(-2, 2),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == true
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-1.png"), fig)

            agent = CircleAgent(Point(0, 0), 0.5)
            pose = Pose(0, 0, 0)
            obstacle = SquareObstacle(Point(0, 0), 1.5, -30)
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

            agent = CircleAgent(Point(0, 0), 0.5)
            pose = Pose(1, 1, 0)
            obstacle = SquareObstacle(Point(0, 0), 1.5, -30)
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

            agent = CircleAgent(Point(0, 0), 0.5)
            pose = Pose(1, 0, 0)
            obstacle = SquareObstacle(Point(0, 0), 1.5, -30)
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-4.png"), fig)

            agent = CircleAgent(Point(0, 0), 1.5)
            pose = Pose(0, 0, 0)
            obstacle = SquareObstacle(Point(0, 0), 1.5, 45)
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

    testset_agent_name = "PolygonAgent"
    @testset "$testset_agent_name" begin
        testset_obstacle_name = "CircleObstacle"
        @testset "$testset_obstacle_name" begin
            agent = PolygonAgent((Point(-1, -1), Point(-1, 1), Point(1, 1), Point(1, -1)))
            pose = Pose(-1, 1, 0)
            obstacle = CircleObstacle(Point(2, 0), 1)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-3, 4),
                Range(-2, 3),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == true
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-1.png"), fig)

            agent = PolygonAgent((Point(-1, -1), Point(-1, 1), Point(1, 1), Point(1, -1)))
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-2.png"), fig)

            agent = PolygonAgent((Point(-1, -1), Point(-1, 1), Point(1, 1), Point(1, -1)))
            pose = Pose(1, 1, 0)
            obstacle = CircleObstacle(Point(2, 0), 1)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-1, 4),
                Range(-2, 3),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == false
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-3.png"), fig)

            agent = PolygonAgent((Point(-1, -1), Point(-1, 1), Point(1, 1), Point(1, -1)))
            pose = Pose(1, 0, 0)
            obstacle = CircleObstacle(Point(2, 0), 1)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-1, 4),
                Range(-2, 2),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == false
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-4.png"), fig)

            agent = PolygonAgent((Point(-1, -1), Point(-1, 1), Point(1, 1), Point(1, -1)))
            pose = Pose(0, 0, 0)
            obstacle = CircleObstacle(Point(0, 0), 3)
            world = World(
                "Test Collisions",
                pose,
                Pose(100, 0, 0),
                Range(-4, 4),
                Range(-4, 4),
                (obstacle,)
            )
            @test is_collision_free(agent, pose, obstacle) == false
            fig = plot_environment(agent, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-5.png"), fig)

            agent = PolygonAgent((Point(-1, -1), Point(-1, 1), Point(1, 1), Point(1, -1)))
            pose = Pose(0, 0, 0)
            obstacle = CircleObstacle(Point(0, 0), 0.5)
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
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-6.png"), fig)

            range = 2
            size_min = 0.5
            size_max = 2
            radius_max = 1
            x1 = rand() * 2 * range - range
            y1 = rand() * 2 * range - range
            x2 = rand() * 2 * range - range
            y2 = rand() * 2 * range - range
            a = rand() * (size_max - size_min) + size_min
            b = rand() * (size_max - size_min) + size_min
            radius = rand() * (radius_max - size_min) + size_min
            angle = rand() * 180
            angle_deg = deg2rad(angle)

            point1 = Point(x1, y1)
            point2 = Point(x2, y2)
            pose1 = Pose(x1, y1, angle_deg)
            pose2 = Pose(x2, y2, angle_deg)

            obstacle_rectangle1 = RectangleObstacle(point1, a, b, angle)
            obstacle_rectangle2 = RectangleObstacle(point2, a, b, angle)
            
            rectangle_noangle1 = RectangleObstacle(Point(0, 0), a, b, 0)
            agent_rectangle1 = PolygonAgent(rectangle_noangle1.vertices)

            rectangle_noangle2 = RectangleObstacle(Point(0, 0), a, b, 0)
            agent_rectangle2 = PolygonAgent(rectangle_noangle2.vertices)
            
            agent_circle1 = CircleAgent(point1, radius)
            agent_circle2 = CircleAgent(point2, radius)

            obstacle_circle1 = CircleObstacle(point1, radius)
            obstacle_circle2 = CircleObstacle(point2, radius)

            @test is_collision_free(agent_rectangle1, pose1, obstacle_circle2) == is_collision_free(agent_circle2, pose2, obstacle_rectangle1)
            @test is_collision_free(agent_rectangle2, pose2, obstacle_circle1) == is_collision_free(agent_circle1, pose1, obstacle_rectangle2)

            world = World(
                "Test Collisions",
                pose1,
                Pose(100, 0, 0),
                Range(-range, range),
                Range(-range, range),
                (obstacle_circle2,)
            )
            fig = plot_environment(agent_rectangle1, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-random-1.png"), fig)

            world = World(
                "Test Collisions",
                pose2,
                Pose(100, 0, 0),
                Range(-range, range),
                Range(-range, range),
                (obstacle_circle1,)
            )
            fig = plot_environment(agent_rectangle2, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-random-2.png"), fig)

            world = World(
                "Test Collisions",
                pose1,
                Pose(100, 0, 0),
                Range(-range, range),
                Range(-range, range),
                (obstacle_rectangle2,)
            )
            fig = plot_environment(agent_circle1, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-random-3.png"), fig)

            world = World(
                "Test Collisions",
                pose2,
                Pose(100, 0, 0),
                Range(-range, range),
                Range(-range, range),
                (obstacle_rectangle1,)
            )
            fig = plot_environment(agent_circle2, world)
            save(joinpath("plots", "$testset_agent_name-$testset_obstacle_name-random-4.png"), fig)

            for _ in 1:1000
                range = 2
                size_min = 0.5
                size_max = 2
                radius_max = 1
                x1 = rand() * 2 * range - range
                y1 = rand() * 2 * range - range
                x2 = rand() * 2 * range - range
                y2 = rand() * 2 * range - range
                a = rand() * (size_max - size_min) + size_min
                b = rand() * (size_max - size_min) + size_min
                radius = rand() * (radius_max - size_min) + size_min
                angle = rand() * 180
                angle_deg = deg2rad(angle)

                point1 = Point(x1, y1)
                point2 = Point(x2, y2)
                pose1 = Pose(x1, y1, angle_deg)
                pose2 = Pose(x2, y2, angle_deg)

                obstacle_rectangle1 = RectangleObstacle(point1, a, b, angle)
                obstacle_rectangle2 = RectangleObstacle(point2, a, b, angle)
                
                rectangle_noangle1 = RectangleObstacle(Point(0, 0), a, b, 0)
                agent_rectangle1 = PolygonAgent(rectangle_noangle1.vertices)

                rectangle_noangle2 = RectangleObstacle(Point(0, 0), a, b, 0)
                agent_rectangle2 = PolygonAgent(rectangle_noangle2.vertices)
                
                agent_circle1 = CircleAgent(point1, radius)
                agent_circle2 = CircleAgent(point2, radius)

                obstacle_circle1 = CircleObstacle(point1, radius)
                obstacle_circle2 = CircleObstacle(point2, radius)

                @test is_collision_free(agent_rectangle1, pose1, obstacle_circle2) == is_collision_free(agent_circle2, pose2, obstacle_rectangle1)
                @test is_collision_free(agent_rectangle2, pose2, obstacle_circle1) == is_collision_free(agent_circle1, pose1, obstacle_rectangle2)

            end

        end
    end
end