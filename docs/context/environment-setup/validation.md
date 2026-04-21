# Validation: WHL test entry

Purpose: 记录用于验证仓库构建与运行环境的标准命令。

Commands:

- 非交互（推荐，用于 CI 与自动化验证）：

    bash docker/scripts/whl.sh start test

  说明：此命令以非交互方式启动 test 模式容器并检查核心服务是否运行。若此命令成功返回“Container is running.”，表示环境准备就绪。

- 交互进入（手动诊断）：

    bash docker/scripts/whl.sh enter test

  说明：在容器已启动后使用此命令进入容器交互 shell 做进一步诊断。优先使用 `start` 来确保可重复的非交互验证流程。

Notes:

- 不要将 docker/.env.*.local 文件提交到仓库（它们为生成态）。
- 若 `start` 失败，检查主机准备脚本（docker/setup_host/setup_host.sh），或查看运行日志：

    bash docker/scripts/whl.sh status test
    docker compose --project-name $(basename $(pwd)) ps

Last-validated: 2026-04-22
Validated-by: automated run via Copilot
