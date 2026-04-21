# Proto parity: planning_internal.proto (Python pb2)

问题概述

- 现象：C++ 日志显示 lane-borrow 进入/退出，但 Python 订阅器的 HasField 检查返回 absent（无法读到 ADCTrajectory.debug.planning_data.lane_borrow）。
- 根因：Python 端生成的 *_pb2.py 与 C++ 使用的 proto 定义不一致（不同容器或不同 bazel 输出根导致的 pb2 不同步）。

快速修复步骤（在与运行验证相同的 test 容器中执行，示例容器：apollo_test_*）

1. 进入工作目录并初始化 Python/Cyber 环境：

   cd /apollo
   source cyber/setup.bash

2. 重新生成 Python pb2（planning_internal）：

   bazel build //modules/common_msgs/planning_msgs:planning_internal_py_pb2

3. 验证 pb2 中包含 LaneBorrowDebug：

   python3 - <<'PY'
from modules.common_msgs.planning_msgs import planning_internal_pb2 as p
print('LaneBorrowDebug' in p.DESCRIPTOR.message_types_by_name)
PY

4. 如果 bazel 因 /apollo/.cache/bazel 权限失败，选项：
   - 以拥有 /apollo/.cache 的容器用户运行 bazel，或
   - 使用临时输出根：bazel --output_user_root=/tmp/bazel_user_root build //modules/common_msgs/planning_msgs:planning_internal_py_pb2

5. 如果修改同时涉及 C++（例如添加 debug 字段或打印），还需在同一容器中重建并重启 planner：

   bazel build //modules/planning:libplanning_component.so
   /apollo/data/integration_tests/run_in_container.sh stop borrow-lane
   source cyber/setup.bash
   /apollo/data/integration_tests/run_in_container.sh start borrow-lane
   /apollo/data/integration_tests/run_in_container.sh check borrow-lane

注意事项

- 一定在与运行时相同的容器/用户中重建：在另一个容器编译不会改变正在运行进程所加载的库或 pb2。
- 避免在生产日志中保留高频 AINFO 调试打印；使用 gflags（例如 --enable_record_debug）或临时分支进行排查。
- 把这个流程写入 docs/context/knowledge/ 以便日后复现。
