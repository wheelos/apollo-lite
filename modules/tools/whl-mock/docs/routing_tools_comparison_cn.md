# Routing 工具对比：send_routing.py vs routing_request.py

## 概述

`send_routing.py` 和 `routing_request.py` 都是用于通过 Apollo Cyber RT 发送 `RoutingRequest` 消息的工具。本文对比它们的优势、劣势和适用场景。

## 功能对比表

| 功能 | send_routing.py | routing_request.py |
|------|----------------|-------------------|
| 配置文件支持 | ✅ 支持 | ❌ 不支持（硬编码） |
| 命令行参数 | ✅ 完整 CLI | ❌ 无 |
| 动态起点 | ✅ 可选 | ✅ 始终启用 |
| 多航点支持 | ✅ 支持 | ❌ 不支持（仅起点+终点） |
| 发送模式 | 单次/循环/交互 | 发送一次后退出 |
| 服务发现等待 | ✅ 可配置（默认 1.0 秒） | ❌ 不支持 |
| 发送次数控制 | ✅ 支持 | ❌ 不支持 |
| 自定义通道 | ✅ 支持 | ❌ 不支持 |
| 交互模式 | ✅ 支持 | ❌ 不支持 |
| parking_info 支持 | ✅ 支持 | ❌ 不支持 |
| 易用性 | 需要了解 CLI | 直接运行即可 |
| 代码复杂度 | 较高（~380 行） | 较低（~205 行） |

## send_routing.py

### 优势

1. **灵活的配置方式**
   - 从外部 `.pb.txt` 文件加载路由
   - 无需修改代码即可更改路由
   - 支持多航点和停车信息（parking_info）

2. **多种发送模式**
   - 单次发送，带服务发现等待
   - 循环模式，可配置间隔和次数
   - 交互模式，手动控制发送

3. **可靠的投递机制**
   - 内置服务发现等待（默认 1.0 秒），确保下游能收到消息
   - 验证 writer 返回值
   - 关闭前正确清理资源

4. **生产级功能**
   - 可配置的日志级别
   - 自定义通道支持
   - 位姿刷新模式（once/fresh）

### 劣势

1. **复杂度较高**
   - 代码量较大（~380 行）
   - 更多依赖（click、uuid、select）
   - 学习曲线较陡

2. **需要 CLI 知识**
   - 需要了解命令行选项
   - 使用语法相对复杂

### 适用场景

- ✅ 多路由自动化测试
- ✅ 生产环境调试和诊断
- ✅ 性能测试（循环模式 + 次数控制）
- ✅ 交互式开发测试
- ✅ 需要复杂路由的场景（多航点、parking_info）
- ✅ 与测试脚本集成

**使用示例：**
```bash
# 自动化测试：发送 5 次，间隔 2 秒
./send_routing.py -i test_route.txt --loop --interval 2.0 --count 5

# 交互式测试，使用动态起点
./send_routing.py --add-pose --interactive

# 使用自定义通道调试
./send_routing.py -i route.txt -c /apollo/custom_routing
```

## routing_request.py

### 优势

1. **简单易用**
   - 代码量少（~205 行）
   - 仅依赖标准库
   - 易于理解和修改

2. **零配置**
   - 直接运行脚本即可
   - 无需记忆命令行参数
   - 自动从定位获取起点

3. **自动退出**
   - 发送一次后干净退出
   - 适合一次性操作

### 劣势

1. **不够灵活**
   - 终点硬编码在源码中
   - 更改路由必须修改代码
   - 不支持基于文件的配置

2. **无服务发现等待**
   - 如果 Routing 模块未就绪，首次发送可能失败
   - 无重试机制
   - 生产环境可靠性较低

3. **功能有限**
   - 仅支持起点+终点（无中间航点）
   - 不支持 parking_info
   - 无循环或交互模式
   - 通道名称固定

### 适用场景

- ✅ 固定终点的快速一次性测试
- ✅ 学习 Cyber RT 基础
- ✅ 嵌入其他脚本
- ✅ 简单验证测试
- ✅ 有简单路由需求的开发环境

**使用示例：**
```bash
# 直接运行
python3 routing_request.py
```

**要更改行为，需编辑源码：**
```python
ORIGINAL_END_X = -3.886315019772555  # 修改终点
ORIGINAL_END_Y = 37.61260329902406
EXTEND_LENGTH = 5.23 / 2              # 修改延伸距离
```

## 决策矩阵

| 场景 | 推荐工具 | 原因 |
|------|----------|------|
| 生产测试 | send_routing.py | 可靠投递，灵活配置 |
| 快速验证 | routing_request.py | 零配置，直接运行 |
| 自动化测试套件 | send_routing.py | 循环模式+次数控制，文件配置 |
| 学习 Cyber RT | routing_request.py | 代码简单，易于理解 |
| 复杂路由 | send_routing.py | 多航点、parking_info 支持 |
| 调试特定通道 | send_routing.py | 自定义通道支持 |
| 交互式开发 | send_routing.py | 交互模式，位姿刷新选项 |
| 一次性固定路由 | routing_request.py | 简单，自动退出 |

## 已知问题

### 服务发现时序问题（两个工具）

Cyber RT 的服务发现机制需要时间来注册 writer，之后消息才能被订阅者接收。

**send_routing.py**：✅ **已解决**
```python
if wait > 0:
    time.sleep(wait)  # 默认 1.0 秒
```

**routing_request.py**：❌ **未处理**
```python
# 发布前无等待 - 首次发送可能失败
```

**对 routing_request.py 的建议：**
在创建 writer 后添加等待：
```python
writer = node.create_writer(ROUTING_REQUEST_TOPIC, RoutingRequest)
time.sleep(1.0)  # 允许服务发现
```

## 总结

| 工具 | 最适合 | 权衡 |
|------|--------|------|
| **send_routing.py** | 生产/测试/自动化 | 以复杂度换取可靠性和灵活性 |
| **routing_request.py** | 快速实验/学习 | 以有限功能换取简单性 |

**结论：**
- 生产环境、测试和任何注重可靠性的场景使用 **send_routing.py**
- 快速一次性测试和学习目的使用 **routing_request.py**
