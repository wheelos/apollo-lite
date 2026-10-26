# Skills

这里保存任务执行方法；仓库事实、模块契约与命令的唯一持久知识权威仍是
`ad_agents/context/`。按任务选择入口，不一次加载所有 skill。

## 工程开发入口

| 任务 | Skill | 交付边界 |
| --- | --- | --- |
| 技术选型、工程框架与架构设计 | [engineering-solution-design](engineering-solution-design/SKILL.md) | 有证据的取舍、原则、边界与最小闭环路线 |
| 算法建模、实现与模块集成 | [algorithm-development](algorithm-development/SKILL.md) | 契约、基线、实现、正确性与效果证据 |
| 延迟、CPU、GPU、内存或吞吐问题 | [performance-optimization](performance-optimization/SKILL.md) | 基线、采集、瓶颈归因、优化及同条件复测 |
| 失败、异常行为、原因未知 | [issue-localization](issue-localization/SKILL.md) | 先检索和取证，再定位；修复需根因证据 |
| 回放、闭环仿真、场景回归 | [simulation-regression](simulation-regression/SKILL.md) | 明确执行模型、确定性断言和证据包 |

从当前问题进入，需要交接时再加载下一项，不把它们串成每次必走的流水线。
明确的小改动直接使用已有契约；不要求重新调研、重做架构或全栈仿真。
问题定位中发现资源瓶颈时转性能分析；算法行为改变时按影响范围选择回归。

## 已有工程执行入口

- [build](build/SKILL.md)：编译与构建问题。
- [lint](lint/SKILL.md)：格式与静态检查。
- [container-build](container-build/SKILL.md)：开发容器镜像。
- [multi-repo-dev](multi-repo-dev/SKILL.md)：多仓库依赖与发布。
- [vendor](vendor/SKILL.md)：离线源码交付。
- [repository-agent-setup](repository-agent-setup/SKILL.md)：agent 布局与技能维护。

命令按需从以下权威入口读取，并核对当前源码或工具帮助；文档引用缺失时
先在 context 下查找同名文件，仍未找到则报告缺口，不发明命令或补装工具：

- `ad_agents/context/framework/build/build-and-test-command-registry.md`
- `ad_agents/context/framework/run/tool-and-artifact-registry.md`
- `ad_agents/context/framework/validation/test-strategy-matrix.md`

## 已有 context 流程的适配边界

本目录的诊断、性能和回归 skill 复用以下文档的方法；领域细节按需读取，
不复制仓库知识。以下适配是有意的，不要求再执行旧流程中的机械步骤：

- `ad_agents/context/framework/skills/issue-localization.md`：保留首错层与因果链，
  按证据建立必要假设，不强制凑足三个假设或遍历所有规划故障分类。
- `ad_agents/context/framework/skills/performance-optimization.md`：保留先采集后优化；
  其中引用的 `performance-testing.md` 当前未找到。本目录流程自含对照测量步骤，
  验证范围使用现有 test strategy matrix，不依赖该缺失文件。
- `ad_agents/context/framework/skills/simulation-debugging.md`：保留执行模型选择、
  配置冻结与首错帧分析，补充开环与闭环证据边界、断言和不稳定性处理。

## 外部方法依据

查看日期：2026-10-26。仅借鉴经阅读确认的方法并作原创归纳，不安装外部 skill，
不把外部指令当成本仓库规则，不引入其脚本、依赖或自动提交行为。

| 原始资料 | 借鉴 | 不直接照搬 |
| --- | --- | --- |
| [Superpowers systematic-debugging](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/systematic-debugging/SKILL.md) | 先取证、找工作样例、反向追踪数据、单假设实验 | 固定重试次数、先加日志再判断是否需要、全量阅读无关实现 |
| [Superpowers verification-before-completion](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/verification-before-completion/SKILL.md) | 声明与实际验证证据一致 | 为每次报告重复运行已通过且仍有效的验证 |
| [Superpowers brainstorming](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/brainstorming/SKILL.md) | 先读现状、澄清真正影响方案的信息、对比可行路径 | 所有任务强制问答、创建并提交设计文档、强制派生 agent |
| [Google Benchmark user guide](https://github.com/google/benchmark/blob/0a78b5d203283b54f65590ecef23e0b835b4787c/docs/user_guide.md) | 预热、重复采样、CPU/墙钟区分与交错对照 | 自动引入 benchmark 库、改变共享主机频率或权限 |
| [SciPy test tips](https://github.com/scipy/scipy/blob/e4ae5a6fb55455b06bbf7ada6cbfa947e6da5529/doc/source/dev/contributor/writing_test_tips.rst) | 浮点近似比较与整数精确比较应区分 | 引入 SciPy 的断言 API 或任意统一浮点容差 |

设计选型另参考其 skill 中列出的 MADR、arc42 与 OpenSSF 原始资料。
自动驾驶算法契约、回放边界和安全限制是结合任务场景的补充，
不宣称上述通用开源 skill 已提供自动驾驶领域验证。
