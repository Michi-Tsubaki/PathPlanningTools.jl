### A Pluto.jl notebook ###
# v0.20.4

using Markdown
using InteractiveUtils

# ╔═╡ 70ad29cc-95a8-4de0-9953-19821d51c7db
using Pkg

# ╔═╡ 426d7377-9d6c-44de-96f1-edd3820e50b5
Pkg.activate("..")

# ╔═╡ fd765e4a-17b4-43d8-a62c-c8c168b18bbe
using LinearAlgebra, Plots

# ╔═╡ 8fe1ba52-d4c7-4dfd-a8d0-46d6d3ffd8fd
using PlutoUI

# ╔═╡ f20a66b4-0177-11f0-31a1-25c46f6480d3
using PathPlanningTools

# ╔═╡ 9dc0c995-4978-4799-a4b5-5265cc8530ca
md"""
## 経路計画をする前に，空間の定義・描画をしてみる
"""

# ╔═╡ 5dc3ecee-f043-43a8-8ad4-ffac25431f8a
gr()

# ╔═╡ 9f7d9099-b321-4268-bc16-680cbc290ded
md"""
### Step1: 離散空間・エージェントを定義する．
"""

# ╔═╡ e69107ab-d46b-406c-851d-ca526dc5d722
example_space = DiscreteSpace(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0, 50, 50, 50)

# ╔═╡ 880b9711-cc1d-4949-98de-eca71e00dc9f
example_agent = Agent(Point(-4.0, -4.0, -4.0), Point(4.0, 4.0, 4.0))

# ╔═╡ 1aa35087-4aee-4c95-98f3-8f80f3cc8a89
md"""
### Step2: ヒューリスティック関数にゴールまでの距離を入れてみる．（描画はされない）
"""

# ╔═╡ b0d30fb1-458f-4652-b57f-1a2f4e1d46fa
for i in 1:example_space.nx
    for j in 1:example_space.ny
        for k in 1:example_space.nz
            x = example_space.x_range[i]
            y = example_space.y_range[j]
            z = example_space.z_range[k]
            here = Point(x,y,z)
            dist = distance(here, example_agent.target_state)
            set_values!(example_space, i, j, k, 0.0, dist, 0.0)
        end
    end
end

# ╔═╡ 62450272-3f09-4896-8590-aec4d0b9e45c
# 値を確認する．
get_values(example_space, 10, 10, 10)

# ╔═╡ e0739ef8-a187-4c0b-9b4c-5d5b14ee4ace
md"""
### Step3: 障害物を設置する．
"""

# ╔═╡ d9d2feb9-be33-40c2-9467-f54b510710c5
begin
obstacles = AbstractObstacle[]
	# 球状の障害物を追加
	# SphereObstacle(中心座標, 半径)
	sphere1 = SphereObstacle((0.0, 0.0, 0.0), 1.0)  # 原点に半径1の球
	push!(obstacles, sphere1)
	
	# 別の球状障害物を追加
	sphere2 = SphereObstacle((2.0, 2.0, 2.0), 0.8)  # 座標(2,2,2)に半径0.8の球
	push!(obstacles, sphere2)
	
	# 直方体の障害物を追加
	# CuboidObstacle(中心座標, (幅, 奥行き, 高さ))
	cuboid1 = CuboidObstacle((-2.0, -2.0, 0.0), (1.0, 1.0, 3.0))  # 座標(-2,-2,0)に1x1x3の直方体
	push!(obstacles, cuboid1)
end

# ╔═╡ 0e2fec53-cb8b-4c11-a4f9-2216d931b0fe
md"""
### Step4: ダミーの経路を用意する．（描画のため）
"""

# ╔═╡ 507c553a-275e-4055-b64c-695f5020a228
# 経路探索の結果（実際にはここに経路計算のコードが入る）
trajectory = [
    Point(-4.0, -4.0, -4.0),
    Point(-2.0, -2.0, -2.0),
    Point(0.0, 0.0, 0.0),
    Point(2.0, 2.0, 2.0),
    Point(4.0, 4.0, 4.0)
]

# ╔═╡ 26ef5688-cb39-4165-b906-83b31e92ad45
md"""
### Step5: 描画する
"""

# ╔═╡ efc0ea4b-ff67-4775-834c-44bdd227fc91
plot = visualize_path([trajectory], obstacles, [example_agent], example_space)

# ╔═╡ 9dd6a301-87cb-44f8-8725-09b9b705b48c
anim = animate_trajectory([trajectory], obstacles, [example_agent], example_space, "draw_environment.gif")

# ╔═╡ 0ece985e-e2c5-451e-85c7-1eb873be92c8
PlutoUI.LocalResource("draw_environment.gif")

# ╔═╡ Cell order:
# ╟─9dc0c995-4978-4799-a4b5-5265cc8530ca
# ╠═70ad29cc-95a8-4de0-9953-19821d51c7db
# ╠═426d7377-9d6c-44de-96f1-edd3820e50b5
# ╠═fd765e4a-17b4-43d8-a62c-c8c168b18bbe
# ╠═8fe1ba52-d4c7-4dfd-a8d0-46d6d3ffd8fd
# ╠═5dc3ecee-f043-43a8-8ad4-ffac25431f8a
# ╠═f20a66b4-0177-11f0-31a1-25c46f6480d3
# ╟─9f7d9099-b321-4268-bc16-680cbc290ded
# ╠═e69107ab-d46b-406c-851d-ca526dc5d722
# ╠═880b9711-cc1d-4949-98de-eca71e00dc9f
# ╟─1aa35087-4aee-4c95-98f3-8f80f3cc8a89
# ╠═b0d30fb1-458f-4652-b57f-1a2f4e1d46fa
# ╠═62450272-3f09-4896-8590-aec4d0b9e45c
# ╟─e0739ef8-a187-4c0b-9b4c-5d5b14ee4ace
# ╠═d9d2feb9-be33-40c2-9467-f54b510710c5
# ╟─0e2fec53-cb8b-4c11-a4f9-2216d931b0fe
# ╠═507c553a-275e-4055-b64c-695f5020a228
# ╟─26ef5688-cb39-4165-b906-83b31e92ad45
# ╠═efc0ea4b-ff67-4775-834c-44bdd227fc91
# ╠═9dd6a301-87cb-44f8-8725-09b9b705b48c
# ╠═0ece985e-e2c5-451e-85c7-1eb873be92c8
