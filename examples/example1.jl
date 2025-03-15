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
example_space = DiscreteSpace(0.0, 10.0, 0.0, 10.0, 0.0, 10.0, 100, 100, 100)

# ╔═╡ 880b9711-cc1d-4949-98de-eca71e00dc9f
example_agents = agents = [
        Agent(Point(1.0, 1.0, 1.0), Point(9.0, 9.0, 9.0), radius=0.2, color="blue"),
        Agent(Point(9.0, 1.0, 1.0), Point(1.0, 9.0, 9.0), radius=0.2, color="red"),
        Agent(Point(1.0, 9.0, 1.0), Point(9.0, 1.0, 9.0), radius=0.2, color="green")
    ]

# ╔═╡ 1aa35087-4aee-4c95-98f3-8f80f3cc8a89
md"""
### Step2: ヒューリスティック関数にゴールまでの距離を入れてみる．（描画はされない）
"""

# ╔═╡ 62450272-3f09-4896-8590-aec4d0b9e45c
# 値を確認する．
get_values(example_space, 10, 10, 10)

# ╔═╡ e0739ef8-a187-4c0b-9b4c-5d5b14ee4ace
md"""
### Step3: 障害物を設置する．
"""

# ╔═╡ 89038698-8d2b-4504-9e70-0c1f563f86a1
obstacles = [
	CuboidObstacle((5.0, 5.0, 5.0), (3.0, 3.0, 3.0)),
        # その他の小さな障害物
    SphereObstacle((3.0, 7.0, 4.0), 1.0),
    SphereObstacle((7.0, 3.0, 6.0), 1.0)
]

# ╔═╡ 0e2fec53-cb8b-4c11-a4f9-2216d931b0fe
md"""
### Step4: ダミーの経路を用意する．（描画のため）
"""

# ╔═╡ 507c553a-275e-4055-b64c-695f5020a228
# 経路探索の結果（実際にはここに経路計算のコードが入る）
trajectories = navigate_agents(example_space, example_agents, obstacles, 1000, 0.3)

# ╔═╡ 0ed32d98-f450-4747-890a-e708674aa861
for (i, trajectory) in enumerate(trajectories)
        println("エージェント$i: $(length(trajectory))ステップ, 総移動距離: $(round(calculate_path_length(trajectory), digits=2))")
end

# ╔═╡ 26ef5688-cb39-4165-b906-83b31e92ad45
md"""
### Step5: 描画する
"""

# ╔═╡ 9dd6a301-87cb-44f8-8725-09b9b705b48c
anim = animate_trajectory(trajectories, obstacles, example_agents, example_space, "multi_agent_navigation.gif")

# ╔═╡ 0ece985e-e2c5-451e-85c7-1eb873be92c8
PlutoUI.LocalResource("multi_agent_navigation.gif")

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
# ╠═62450272-3f09-4896-8590-aec4d0b9e45c
# ╟─e0739ef8-a187-4c0b-9b4c-5d5b14ee4ace
# ╠═89038698-8d2b-4504-9e70-0c1f563f86a1
# ╟─0e2fec53-cb8b-4c11-a4f9-2216d931b0fe
# ╠═507c553a-275e-4055-b64c-695f5020a228
# ╠═0ed32d98-f450-4747-890a-e708674aa861
# ╟─26ef5688-cb39-4165-b906-83b31e92ad45
# ╠═9dd6a301-87cb-44f8-8725-09b9b705b48c
# ╠═0ece985e-e2c5-451e-85c7-1eb873be92c8
