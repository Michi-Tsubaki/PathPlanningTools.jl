#=
3次元空間を適切な幅で離散化する．
- 連続な場を離散的に見ることで，局所計算を容易にする．
- グラフ構造を用意することで，グラフ探査を容易にする．
=#

module PathPlanningTools

using Plots

export PointValues, Point, DiscreteSpace, AbstractAgent, Agent, distance, AbstractObstacle, CuboidObstacle, SphereObstacle, contains, set_values!, get_values, visualize_path, draw_cuboid!, draw_sphere!

# 各点の持つ値
struct PointValues
    potential::Float64 # ポテンシャル
    heuristic::Float64 # ヒューリスティック関数の値
    obstacle::Bool  # 障害物の中か（障害物の中ならばTrue）
end

# 3次元空間の座標
struct Point
    x::Float64
    y::Float64
    z::Float64
end

# 3次元空間の定義
struct DiscreteSpace
    nx::Int
    ny::Int
    nz::Int
    dx::Float64
    dy::Float64
    dz::Float64
    x_range::LinRange
    y_range::LinRange
    z_range::LinRange
    data::Array{PointValues, 3}
end

function DiscreteSpace(x_min, x_max, y_min, y_max, z_min, z_max, nx, ny, nz)
    dx = (x_max - x_min) / nx
    dy = (y_max - y_min) / ny
    dz = (z_max - z_min) / nz

    x_range = LinRange(x_min, x_max, nx)
    y_range = LinRange(y_min, y_max, ny)
    z_range = LinRange(z_min, z_max, nz)

    data = Array{PointValues, 3}(undef, nx, ny, nz)
    for i in 1:nx, j in 1:ny, k in 1:nz
        data[i, j, k] = PointValues(0.0, 0.0, 0.0)
    end

    return DiscreteSpace(nx, ny, nz, dx, dy, dz, x_range, y_range, z_range, data)
end

# エージェントの定義
# 一般のエージェント（将来色々な形のエージェントに拡張するため）
abstract type AbstractAgent end

# 球型のエージェント
mutable struct Agent <: AbstractAgent
    state::Tuple{Float64, Float64, Float64} #中心座標（Pointではないので注意）
    radius::Float64
    initial_state::Point
    target_state::Point
    color::String
end
                    
function Agent(initial_state::Point, target_state::Point; radius::Float64=0.1, color::String="blue")
    state = (initial_state.x, initial_state.y, initial_state.z)
    return Agent(state, radius, initial_state, target_state, color)
end

# 補助関数（距離）
function distance(a::Point, b::Point)
    return sqrt((a.x-b.x)^2 + (a.y-b.y)^2 + (a.z-b.z)^2 )
end

# 障害物の定義
abstract type AbstractObstacle end

# 球型の障害物
struct SphereObstacle <: AbstractObstacle
    center::Tuple{Float64, Float64, Float64} # centroid(x,y,z)
    radius::Float64 # radius
end

function contains(obstacle::SphereObstacle, agent::Agent, point::Point)
    center_point = Point(obstacle.center[1], obstacle.center[2], obstacle.center[3])
    dist = distance(center_point, point)
    return dist <= obstacle.radius + agent.radius
end

# 直方体の障害物
struct CuboidObstacle <: AbstractObstacle
    center::Tuple{Float64, Float64, Float64}  # center(x,y,z)
    size::Tuple{Float64, Float64, Float64}  # width(x), depth(y), height(z)
end

function contains(obstacle::CuboidObstacle, agent::Agent, point::Point)
    cx, cy, cz = obstacle.center
    sx, sy, sz = obstacle.size
    
    # 直方体の境界を計算
    half_width = sx / 2
    half_depth = sy / 2
    half_height = sz / 2
    
    x_min = cx - half_width
    x_max = cx + half_width
    y_min = cy - half_depth
    y_max = cy + half_depth
    z_min = cz - half_height
    z_max = cz + half_height
    
    # 半径も考慮
    return (x_min - agent.radius <= point.x <= x_max + agent.radius &&
            y_min - agent.radius <= point.y <= y_max + agent.radius &&
            z_min - agent.radius <= point.z <= z_max + agent.radius)
end

# 各微小空間の値
function set_values!(space::DiscreteSpace, i, j, k, val1, val2, val3)
    if 1 <= i <= space.nx && 1 <= j <= space.ny && 1 <= k <= space.nz
        space.data[i, j, k] = PointValues(val1, val2, val3)
    else
        error("Out of index")
    end
end

function get_values(space::DiscreteSpace, i, j, k)
    if 1 <= i <= space.nx && 1 <= j <= space.ny && 1 <= k <= space.nz
        return space.data[i, j, k]
    else
        error("Error: Out of index")
    end
end

# 描画関数
function visualize_path(trajectory::Vector{Point}, obstacles::Vector{<:AbstractObstacle}, 
    agent::Agent, space::DiscreteSpace)

    # 描画の初期化
    plt = plot3d(
        xlim = (space.x_range[1], space.x_range[end]),
        ylim = (space.y_range[1], space.y_range[end]),
        zlim = (space.z_range[1], space.z_range[end]),
        xlabel = "X",
        ylabel = "Y",
        zlabel = "Z",
        title = "Path Planner",
        legend = true,
        size = (800, 600),
        camera = (30, 30)  # 視点
    )

    # 軌跡
    trajectory_x = [p.x for p in trajectory]
    trajectory_y = [p.y for p in trajectory]
    trajectory_z = [p.z for p in trajectory]

    plot3d!(plt, trajectory_x, trajectory_y, trajectory_z, 
            line = (:blue, 2), 
            label = "Agent Trajectory"
        )

    # スタート地点
    scatter3d!(plt, [agent.initial_state.x], [agent.initial_state.y], [agent.initial_state.z], 
    marker = (:circle, 8, :green), label = "Start")

    # ゴール地点
    scatter3d!(plt, [agent.target_state.x], [agent.target_state.y], [agent.target_state.z], 
    marker = (:circle, 8, :red), 
    label = "Goal")

    # 障害物の描画
    for obstacle in obstacles
        if isa(obstacle, SphereObstacle)
            # 球形障害物 - 点で表現
            draw_sphere!(plt, obstacle)
        elseif isa(obstacle, CuboidObstacle)
            # 立方体障害物
            draw_cuboid!(plt, obstacle)
        end
    end

    return plt
end


function draw_cuboid!(p, cube::CuboidObstacle; color=:black, alpha=1.0, linewidth=1.5, fillalpha=0.3)
    xmin, xmax = cube.center[1] - cube.size[1]/2, cube.center[1] + cube.size[1]/2
    ymin, ymax = cube.center[2] - cube.size[2]/2, cube.center[2] + cube.size[2]/2
    zmin, zmax = cube.center[3] - cube.size[3]/2, cube.center[3] + cube.size[3]/2

    vertices_x = [xmin, xmax, xmin, xmax, xmin, xmax, xmin, xmax]
    vertices_y = [ymin, ymin, ymax, ymax, ymin, ymin, ymax, ymax]
    vertices_z = [zmin, zmin, zmin, zmin, zmax, zmax, zmax, zmax]
    
    connections = [
        (1,2,6,5), # front
        (3,7,8,4), # back
        (1,5,7,3), # left
        (2,4,8,6), # right
        (1,2,4,3), # bottom
        (5,6,8,7)  # top
    ]
    
    plot!(p, vertices_x, vertices_y, vertices_z,
          seriestype=:mesh3d,
          connections=connections,
          color=color,
          alpha=fillalpha,
          linecolor=:gray,
          linewidth=linewidth,
          label="Cuboid Obstacle")
end

function draw_sphere!(p, sphere::SphereObstacle; color="black", alpha=1.0, label="")
    radius = sphere.radius
    position = sphere.center
    u = range(0, 2π, length=30)
    v = range(0, π, length=30)
    
    x = position[1] .+ radius * [cos(θ) * sin(φ) for θ in u, φ in v]
    y = position[2] .+ radius * [sin(θ) * sin(φ) for θ in u, φ in v]
    z = position[3] .+ radius * [cos(φ) for θ in u, φ in v]
    
    plot3d!(p, x[:], y[:], z[:], color=color, alpha=alpha, label=label, seriestype=:surface)
end

function animate_trajectory(trajectory::Vector{Point}, obstacles::Vector{<:AbstractObstacle}, agent::Agent, space::DiscreteSpace, filename::String="agent_animation.gif")

    n_frames = min(length(trajectory), 500) # 500フレーム以上はカット！
    step_size = max(1, div(length(trajectory), n_frames))

    println("Creating Frame ID: $(n_frames)")

    anim = @animate for i in 1:step_size:length(trajectory)
        current_trajectory = trajectory[1:i] # 現在までの経路
        plt = visualize_path(current_trajectory, obstacles, agent, space)

        current_pos = trajectory[i] # 現在の位置
        scatter3d!(plt, [current_pos.x], [current_pos.y], [current_pos.z], 
        marker = (:star, 8, :blue), 
        label = "Current")

        # step info
        title!("Agent's Path Planning - Step $(i) of $(length(trajectory))")
    end
    gif(anim, filename, fps = 10) 
    println("アニメーションを保存しました．")
    return anim
end


end
