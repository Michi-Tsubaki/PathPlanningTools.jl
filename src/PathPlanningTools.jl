#=
3次元空間を適切な幅で離散化する．
- 連続な場を離散的に見ることで，局所計算を容易にする．
- グラフ構造を用意することで，グラフ探査を容易にする．
=#

module PathPlanningTools

using Plots

export PointValues, Point, DiscreteSpace, AbstractAgent, Agent, distance, AbstractObstacle, CuboidObstacle, SphereObstacle, contains, set_values!, get_values, visualize_path, draw_cuboid!, draw_sphere!, animate_trajectory, navigate_agents, find_next_position_with_agents, random_walk, calculate_repulsive_potential, update_agent!, calculate_path_length
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
    
    half_width = sx / 2
    half_depth = sy / 2
    half_height = sz / 2
    
    x_min = cx - half_width
    x_max = cx + half_width
    y_min = cy - half_depth
    y_max = cy + half_depth
    z_min = cz - half_height
    z_max = cz + half_height
    
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
function visualize_path(trajectories::Vector{Vector{Point}}, obstacles::Vector{<:AbstractObstacle}, 
    agents::Vector{Agent}, space::DiscreteSpace)

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

    # 各エージェントごとに軌跡を描画
    for (i, (trajectory, agent)) in enumerate(zip(trajectories, agents))
        trajectory_x = [p.x for p in trajectory]
        trajectory_y = [p.y for p in trajectory]
        trajectory_z = [p.z for p in trajectory]

        plot3d!(plt, trajectory_x, trajectory_y, trajectory_z, 
                line = (agent.color, 2), 
                label = "Agent $(i) Trajectory"
            )

        # スタート地点
        scatter3d!(plt, [agent.initial_state.x], [agent.initial_state.y], [agent.initial_state.z], 
        marker = (:circle, 8, :green), label = "Start $(i)")

        # ゴール地点
        scatter3d!(plt, [agent.target_state.x], [agent.target_state.y], [agent.target_state.z], 
        marker = (:circle, 8, :red), 
        label = "Goal $(i)")
    end

    # 障害物の描画
    for obstacle in obstacles
        if isa(obstacle, SphereObstacle)
            draw_sphere!(plt, obstacle)
        elseif isa(obstacle, CuboidObstacle)
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
        (1,2,6,5),
        (3,7,8,4),
        (1,5,7,3),
        (2,4,8,6), 
        (1,2,4,3),
        (5,6,8,7)
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

function animate_trajectory(trajectories::Vector{Vector{Point}}, obstacles::Vector{<:AbstractObstacle}, 
    agents::Vector{Agent}, space::DiscreteSpace, filename::String="agents_animation.gif")

    max_length = maximum(length.(trajectories))
    n_frames = min(max_length, 500)
    step_size = max(1, div(max_length, n_frames))

    println("Creating Frame ID: $(n_frames)")

    anim = @animate for i in 1:step_size:max_length
        plt = visualize_path(trajectories, obstacles, agents, space)
        
        # 各エージェントの現在位置をプロット
        for (j, (trajectory, agent)) in enumerate(zip(trajectories, agents))
            if i <= length(trajectory)
                current_pos = trajectory[i]
                scatter3d!(plt, [current_pos.x], [current_pos.y], [current_pos.z], 
                marker = (:star, 8, agent.color), 
                label = "Current $(j)")
            end
        end

        title!("Agents' Path Planning - Step $(i) of $(max_length)")
    end
    gif(anim, filename, fps = 10) 
    println("アニメーションを保存しました．")
    return anim
end

function navigate_agents(space::DiscreteSpace, agents::Vector{Agent}, obstacles::Vector{<:AbstractObstacle}, max_steps::Int=500, potential_weight::Float64=0.5)
    
    # 全エージェントの軌跡
    all_trajectories = Vector{Vector{Point}}(undef, length(agents))
    
    # 各エージェントの経路長を記録するベクトル
    path_lengths = zeros(Float64, length(agents))
    
    # 全エージェントの現在位置
    current_positions = Vector{Point}(undef, length(agents))
    
    # スタックの基準
    space_size_x = space.x_range[end] - space.x_range[1]
    space_size_y = space.y_range[end] - space.y_range[1]
    space_size_z = space.z_range[end] - space.z_range[1]
    max_dimension = max(space_size_x, space_size_y, space_size_z)
    min_move_threshold = max_dimension * 0.001
    history_length = 20
    
    # 初期化
    for (agent_idx, agent) in enumerate(agents)
        current_positions[agent_idx] = Point(agent.state[1], agent.state[2], agent.state[3])
        all_trajectories[agent_idx] = [current_positions[agent_idx]]
    end
    
    # スタック検出のための履歴
    position_histories = [Vector{Point}() for _ in 1:length(agents)]
    for agent_idx in 1:length(agents)
        push!(position_histories[agent_idx], current_positions[agent_idx])
    end
    
    # 訪問地点の記録（エージェントごと）
    visited_positions = [Dict{Tuple{Int,Int,Int}, Int}() for _ in 1:length(agents)]
    
    # 進捗追跡
    step_count = 0
    last_progress = zeros(Int, length(agents))
    best_distances = [distance(current_positions[i], agents[i].target_state) for i in 1:length(agents)]
    best_positions = copy(current_positions)
    initial_distances = copy(best_distances)
    
    # ゴール到達状態
    reached_goal = falses(length(agents))
    
    # メインループ
    while step_count < max_steps && !all(reached_goal)
        # 全エージェントの次の位置を計算（現在の全エージェント位置を考慮）
        next_positions = Vector{Point}(undef, length(agents))
        
        # 各エージェントの次の位置を計算
        for (agent_idx, agent) in enumerate(agents)
            if reached_goal[agent_idx]
                next_positions[agent_idx] = current_positions[agent_idx]
                continue
            end
            
            # 次の位置を計算（他のエージェントも障害物として扱う）
            start_position = Point(agent.initial_state.x, agent.initial_state.y, agent.initial_state.z)
            next_position = find_next_position_with_agents(
                space, agent, agent_idx, obstacles, start_position, 
                path_lengths[agent_idx], visited_positions[agent_idx], 
                current_positions, agents, potential_weight
            )
            
            next_positions[agent_idx] = next_position
        end
        
        # 全エージェントの位置を更新
        for (agent_idx, agent) in enumerate(agents)
            if reached_goal[agent_idx]
                continue
            end
            
            # 現在位置と次の位置の距離
            move_distance = distance(current_positions[agent_idx], next_positions[agent_idx])
            
            # 経路長を更新
            path_lengths[agent_idx] += move_distance
            
            # ゴールへの距離
            current_distance_to_goal = distance(next_positions[agent_idx], agent.target_state)
            
            # ゴールに近づいたか記録
            if current_distance_to_goal < best_distances[agent_idx]
                best_distances[agent_idx] = current_distance_to_goal
                best_positions[agent_idx] = next_positions[agent_idx]
                last_progress[agent_idx] = step_count
            end
            
            # 長時間進展がない場合
            if step_count - last_progress[agent_idx] > 50
                # 前回の良い位置に戻る
                println("エージェント$(agent_idx): 長時間進展がないため、前回の良い位置まで戻ります。ステップ: $step_count")
                update_agent!(agent, best_positions[agent_idx])
                current_positions[agent_idx] = best_positions[agent_idx]
                push!(all_trajectories[agent_idx], current_positions[agent_idx])
                
                # そこからランダムウォーク
                println("エージェント$(agent_idx): 前回の良い位置からランダムウォークを開始します。")
                for _ in 1:10
                    random_pos = random_walk(space, agent, obstacles, current_positions[agent_idx])
                    update_agent!(agent, random_pos)
                    current_positions[agent_idx] = random_pos
                    push!(all_trajectories[agent_idx], current_positions[agent_idx])
                end
                
                # 履歴リセット
                position_histories[agent_idx] = [current_positions[agent_idx]]
                last_progress[agent_idx] = step_count
                continue
            end
            
            # 履歴更新
            if length(position_histories[agent_idx]) >= history_length
                popfirst!(position_histories[agent_idx])
            end
            push!(position_histories[agent_idx], next_positions[agent_idx])
            
            # スタック検出
            if length(position_histories[agent_idx]) >= history_length
                center_x = sum(p.x for p in position_histories[agent_idx]) / length(position_histories[agent_idx])
                center_y = sum(p.y for p in position_histories[agent_idx]) / length(position_histories[agent_idx])
                center_z = sum(p.z for p in position_histories[agent_idx]) / length(position_histories[agent_idx])
                center_position = Point(center_x, center_y, center_z)
                
                variance = sum(distance(p, center_position)^2 for p in position_histories[agent_idx]) / length(position_histories[agent_idx])
                spread = sqrt(variance)
                
                if spread < min_move_threshold * 5
                    # スタック検出
                    println("エージェント$(agent_idx): スタック検出: 過去$(history_length)ステップの分散($(round(spread, digits=6)))が小さいため、ランダムウォークを試みます。ステップ: $step_count")
                    
                    # ランダムウォーク
                    for _ in 1:10
                        random_pos = random_walk(space, agent, obstacles, current_positions[agent_idx])
                        update_agent!(agent, random_pos)
                        current_positions[agent_idx] = random_pos
                        push!(all_trajectories[agent_idx], current_positions[agent_idx])
                    end
                    
                    # 履歴リセット
                    position_histories[agent_idx] = [current_positions[agent_idx]]
                    continue
                end
            end
            
            # エージェント位置を更新
            update_agent!(agent, next_positions[agent_idx])
            current_positions[agent_idx] = next_positions[agent_idx]
            
            # 軌跡に追加
            push!(all_trajectories[agent_idx], current_positions[agent_idx])
            
            # ゴール判定
            if current_distance_to_goal < agent.radius + 0.05
                println("エージェント$(agent_idx): ゴールに到達しました！ステップ: $step_count")
                reached_goal[agent_idx] = true
            end
        end
        
        # ステップカウント更新
        step_count += 1
        
        # 進捗表示
        if step_count % 100 == 0
            for (agent_idx, agent) in enumerate(agents)
                if !reached_goal[agent_idx]
                    current_distance = distance(current_positions[agent_idx], agent.target_state)
                    progress = (initial_distances[agent_idx] - current_distance) / initial_distances[agent_idx] * 100
                    println("エージェント$(agent_idx): ステップ: $step_count, ゴールまでの距離: $(round(current_distance, digits=3)), 進捗: $(round(progress, digits=1))%")
                end
            end
        end
    end
    
    # 最終状態表示
    for (agent_idx, agent) in enumerate(agents)
        if !reached_goal[agent_idx]
            println("エージェント$(agent_idx): 最大ステップ数に達しました。ゴールまでの距離: $(round(distance(current_positions[agent_idx], agent.target_state), digits=3))")
        end
    end
    
    return all_trajectories
end

# エージェント間の反発を考慮した次位置計算関数
function find_next_position_with_agents(space::DiscreteSpace, agent::Agent, agent_idx::Int,
                             obstacles::Vector{<:AbstractObstacle}, 
                             start_position::Point,
                             path_length::Float64,
                             visited_positions::Dict{Tuple{Int,Int,Int}, Int},
                             all_agent_positions::Vector{Point},
                             all_agents::Vector{Agent},
                             potential_weight::Float64=0.5,
                             random_walk_prob::Float64=0.1)
    
    # 現在位置の最も近い格子点を見つける
    current_pos = Point(agent.state[1], agent.state[2], agent.state[3])
    i = round(Int, (current_pos.x - space.x_range[1]) / space.dx) + 1
    j = round(Int, (current_pos.y - space.y_range[1]) / space.dy) + 1
    k = round(Int, (current_pos.z - space.z_range[1]) / space.dz) + 1
    
    # 現在位置を訪問済みとしてマーク
    current_cell = (i, j, k)
    visited_positions[current_cell] = get(visited_positions, current_cell, 0) + 1
    
    # ランダムウォーク確率の動的調整
    actual_random_walk_prob = random_walk_prob
    visit_count = visited_positions[current_cell]
    if visit_count > 3
        actual_random_walk_prob = min(0.8, random_walk_prob * visit_count / 2)
    end
    
    # ランダムウォークを実行するかどうか
    if rand() < actual_random_walk_prob
        return random_walk(space, agent, obstacles, current_pos, all_agent_positions, all_agents, agent_idx)
    end
    
    # 隣接するセルを取得 (最大26近傍)
    neighbors = []
    for di in -1:1
        for dj in -1:1
            for dk in -1:1
                if di == 0 && dj == 0 && dk == 0
                    continue  # 現在位置はスキップ
                end
                
                ni, nj, nk = i + di, j + dj, k + dk
                if 1 <= ni <= space.nx && 1 <= nj <= space.ny && 1 <= nk <= space.nz
                    neighbor_point = Point(space.x_range[ni], space.y_range[nj], space.z_range[nk])
                    
                    # 障害物チェック
                    obstacle_free = true
                    for obstacle in obstacles
                        if contains(obstacle, agent, neighbor_point)
                            obstacle_free = false
                            break
                        end
                    end
                    
                    # 他のエージェントとの衝突チェック
                    for (other_idx, other_agent) in enumerate(all_agents)
                        if other_idx != agent_idx  # 自分自身は除外
                            other_pos = all_agent_positions[other_idx]
                            if distance(neighbor_point, other_pos) < (agent.radius + other_agent.radius)
                                obstacle_free = false
                                break
                            end
                        end
                    end
                    
                    if obstacle_free
                        push!(neighbors, (ni, nj, nk, neighbor_point))
                    end
                end
            end
        end
    end
    
    # 各隣接セルのスコアを計算
    best_score = Inf
    best_position = nothing
    
    for (ni, nj, nk, neighbor_point) in neighbors
        # 隣接点までの距離
        step_distance = distance(current_pos, neighbor_point)
        
        # ヒューリスティック: スタートからの実際の経路長 + ゴールまでの予測距離
        distance_from_start = path_length + step_distance  # 実際にたどってきた経路長 + この移動のコスト
        distance_to_goal = distance(neighbor_point, agent.target_state)
        heuristic = distance_from_start + distance_to_goal
        
        # ポテンシャル計算
        # 1. 障害物からの反発力
        repulsive = calculate_repulsive_potential(neighbor_point, obstacles, agent)
        
        # 2. 他のエージェントからの反発力
        agent_repulsive = 0.0
        for (other_idx, other_agent) in enumerate(all_agents)
            if other_idx != agent_idx  # 自分自身は除外
                other_pos = all_agent_positions[other_idx]
                dist = distance(neighbor_point, other_pos) - agent.radius - other_agent.radius
                if dist < 0
                    # エージェント内部 - 進入不可
                    agent_repulsive += 1000.0
                elseif dist < 0.5  # 反発力の影響範囲
                    agent_repulsive += (0.5 - dist) * 20.0  # 近いほど強い反発
                end
            end
        end
        
        # 3. ゴールからの引力
        attractive = distance_to_goal * 5.0
        
        # 4. 訪問ペナルティ
        visit_penalty = get(visited_positions, (ni, nj, nk), 0) * 2.0
        
        # 合計ポテンシャル
        potential = repulsive + agent_repulsive + attractive + visit_penalty
        
        # 重み付けされた総合スコア
        score = (1 - potential_weight) * heuristic + potential_weight * potential
        
        if score < best_score
            best_score = score
            best_position = neighbor_point
        end
    end
    
    # もし動ける場所がなければランダムウォークを試みる
    if best_position === nothing
        return random_walk(space, agent, obstacles, current_pos, all_agent_positions, all_agents, agent_idx)
    end
    
    return best_position
end

# 単一エージェント用のランダムウォーク関数
function random_walk(space::DiscreteSpace, agent::Agent, 
    obstacles::Vector{<:AbstractObstacle}, 
    current_pos::Point)
# ランダムな方向ベクトルを生成
θ = 2π * rand()  # XY平面での角度
φ = π * rand()   # Z方向の角度

# 単位ベクトル
dx = sin(φ) * cos(θ)
dy = sin(φ) * sin(θ)
dz = cos(φ)

# 新しい位置（小さな歩幅）
step_size = 0.05
new_x = current_pos.x + dx * step_size
new_y = current_pos.y + dy * step_size
new_z = current_pos.z + dz * step_size

# 空間の境界内に収める
new_x = clamp(new_x, space.x_range[1], space.x_range[end])
new_y = clamp(new_y, space.y_range[1], space.y_range[end])
new_z = clamp(new_z, space.z_range[1], space.z_range[end])

new_pos = Point(new_x, new_y, new_z)

# 障害物との衝突チェック
for obstacle in obstacles
if contains(obstacle, agent, new_pos)
# 衝突する場合は現在位置を返す
return current_pos
end
end

return new_pos
end

# 他のエージェントも考慮したランダムウォーク
function random_walk(space::DiscreteSpace, agent::Agent, 
                               obstacles::Vector{<:AbstractObstacle}, 
                               current_pos::Point,
                               all_agent_positions::Vector{Point},
                               all_agents::Vector{Agent},
                               agent_idx::Int)
    # ランダムな方向ベクトルを生成
    θ = 2π * rand()  # XY平面での角度
    φ = π * rand()   # Z方向の角度
    
    # 単位ベクトル
    dx = sin(φ) * cos(θ)
    dy = sin(φ) * sin(θ)
    dz = cos(φ)
    
    # 新しい位置（小さな歩幅）
    step_size = 0.05
    new_x = current_pos.x + dx * step_size
    new_y = current_pos.y + dy * step_size
    new_z = current_pos.z + dz * step_size
    
    # 空間の境界内に収める
    new_x = clamp(new_x, space.x_range[1], space.x_range[end])
    new_y = clamp(new_y, space.y_range[1], space.y_range[end])
    new_z = clamp(new_z, space.z_range[1], space.z_range[end])
    
    new_pos = Point(new_x, new_y, new_z)
    
    # 障害物との衝突チェック
    for obstacle in obstacles
        if contains(obstacle, agent, new_pos)
            # 衝突する場合は現在位置を返す
            return current_pos
        end
    end
    
    # 他のエージェントとの衝突チェック
    for (other_idx, other_agent) in enumerate(all_agents)
        if other_idx != agent_idx  # 自分自身は除外
            other_pos = all_agent_positions[other_idx]
            if distance(new_pos, other_pos) < (agent.radius + other_agent.radius)
                # 衝突する場合は現在位置を返す
                return current_pos
            end
        end
    end
    
    return new_pos
end

# 反発ポテンシャルの計算
function calculate_repulsive_potential(point::Point, obstacles::Vector{<:AbstractObstacle}, agent::Agent)
    repulsive = 0.0
    for obstacle in obstacles
        if isa(obstacle, SphereObstacle)
            # 球体障害物の場合
            center = Point(obstacle.center[1], obstacle.center[2], obstacle.center[3])
            dist = distance(center, point) - obstacle.radius - agent.radius
            if dist < 0
                # 障害物内部 - 進入不可
                repulsive += 1000.0
            elseif dist < 0.3  # 反発力の影響範囲
                repulsive += (0.3 - dist) * 10.0  # 近いほど強い反発
            end
        elseif isa(obstacle, CuboidObstacle)
            # 立方体障害物の場合 - 境界からの最小距離を概算
            cx, cy, cz = obstacle.center
            sx, sy, sz = obstacle.size
            
            dx = max(abs(point.x - cx) - sx/2, 0)
            dy = max(abs(point.y - cy) - sy/2, 0)
            dz = max(abs(point.z - cz) - sz/2, 0)
            
            dist = sqrt(dx^2 + dy^2 + dz^2) - agent.radius
            if dist < 0
                repulsive += 1000.0
            elseif dist < 0.3
                repulsive += (0.3 - dist) * 10.0
            end
        end
    end
    return repulsive
end

# 経路の総距離を計算する関数
function calculate_path_length(trajectory::Vector{Point})
    total_length = 0.0
    for i in 2:length(trajectory)
        total_length += distance(trajectory[i-1], trajectory[i])
    end
    return total_length
end

# エージェントの更新関数
function update_agent!(agent::Agent, new_position::Point)
    agent.state = (new_position.x, new_position.y, new_position.z)
end

end
