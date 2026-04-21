# 文档模板（doc-template.md）

---
# Front-matter（请保留 YAML 样式或简单键值清单）
Title: <短标题>
Owner: @github-username
Status: draft | validated | deprecated
Last-validated: YYYY-MM-DD
Validated-by: <CI job 或 人名>

# 快速命令样例（确保命令精确，可复制运行）
Commands:
  build: bazel build --config=opt -- //modules/planning/...
  test: bazel test //modules/planning/...

---

1) What: 一句总结说明该文档涵盖的内容与目的。
2) Why: 说明该内容为何重要，以及应用场景与假设前提。
3) How: 精确可复现的步骤、确切命令、相关配置路径和关键代码位置（相对仓库路径）。
4) Validation: 验证步骤（运行哪个命令、期望输出、如何判断通过）。
5) Related: 关联代码文件、其他文档和历史运行/测试用例路径。

作者注意事项：
- 每个文档保持单一主题；文件名使用 kebab-case。
- 在文件顶部或结尾包含一个 "Last-validated" 字段并在通过验证后更新。
- 在文档中引用具体代码路径或行号，便于审查者复现。

