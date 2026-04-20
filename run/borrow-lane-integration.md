# Borrow-lane headless integration (borrow-lane)

目的

在 headless 环境里复现并验证 borrow-lane（绕障）场景的规划行为，确保 planner 生成 LANE-BORROW 路径且 Python 订阅器能观察到 ADCTrajectory.debug 中的 lane_borrow 字段。

快速复现（容器内）

1. 进入 test 容器并初始化：

   cd /apollo
   source cyber/setup.bash

2. 使用集成测试 harness（会自动 stop/start 并发送路由）：

   /apollo/data/integration_tests/run_in_container.sh check borrow-lane

3. 观察总结：运行结束后 summary 会显示 verdict（期望: borrow-completed）。日志位置与订阅器摘录位于 run 目录下（例：/apollo/data/runs/...）。

常见故障与排查

- 路由失败（"Failed to search route" / "Failed to find nearest lane"）：检查场景 routing 请求是否是双点请求，起点仍位于 `243_1_-3`，终点仍位于 `909_1_-3`；`routing_start` 模式下不要再依赖 `--use-localization-start`。
- Python 读不到 lane_borrow：参见 docs/context/knowledge/proto-parity.md，按步骤重建 planning_internal_py_pb2 并重启 planner（在同一容器）。

运行约束

- 请在当前工作区对应的 `apollo_test_*` 容器里执行这些运行验证命令；不要把验证结果建立在 `apollo_dev_*` 上。
- harness 默认忽略绕障后接近终点导致的 pull-over 问题；把 pull-over 作为独立问题跟踪。

参考

- 数据目录：data/integration_tests/
- 发送工具：data/integration_tests/tools/routing_request_sender.py
- 订阅工具：data/integration_tests/tools/subscribe_planning.py
