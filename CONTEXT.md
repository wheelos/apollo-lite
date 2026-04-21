# Context repository

Remote: git@github.com:wheelos-service/context.git

规则：
每个 session 开始之前，先同步下 context：

  git fetch context
  git merge context/main  # 或者：git pull context main，依据实际分支名

（如需把规则写入独立的 context 仓库，请改为子模块或手动将内容推送到该远端仓库。）
