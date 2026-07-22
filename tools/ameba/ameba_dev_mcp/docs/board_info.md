# `board_info.json5` 配置说明

`<SDK_ROOT>/board_info.json5` 描述测试床上的物理板：每块板一个 alias，记录串口 /
波特率等。MCP 的 flash、串口 tool **都只接收 alias**，
其余参数全部从这里读。

> 模板：[`board_info.template.json5`](board_info.template.json5) — MCP 在文件缺失时
> 自动生成的内容；想跳过自动建模板，直接 copy 到 `<SDK_ROOT>/board_info.json5` 也行。
>
> 第一次调用任意 flash / serial tool 时，如果文件不存在，MCP 会自动建一个空模板并提示路径。

> **Pro2 注意**：AmebaPro2 不支持 AmebaRemoteService，所有板必须直接通过 USB serial
> 连接到本机（`transport` 固定为 `"local"`）。

---

## 1. 顶层结构

```json5
/*
 * board_info.json5 — test-bench config: which SoC is on which serial port.
 */
{
  "schema_version": 1.1,

  "defaults": {
    "baudrate": 115200,
    "monitor_baudrate": 115200,
    "memory_type": "nor"
  },

  "boards": {
    "RTL8735B_COM5": { ... },
    "RTL8735B_COM6": { ... }
  }
}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `schema_version` | number | 当前 `1.1` |
| `defaults.baudrate` | int | 烧录与开端口默认速率（115200 bps） |
| `defaults.monitor_baudrate` | int | 串口监视默认速率 |
| `defaults.memory_type` | `"nor"` / `"nand"` | 默认 flash 类型 |
| `boards` | dict | key 即 alias，每个 board 一个条目 |

---

## 2. alias 命名建议

- 推荐格式：`<SOC>_<PORT>`。
- 例：`RTL8735B_COM5`、`RTL8735B_ttyUSB0`、`RTL8735B_COM6`。
- 不强制；任意唯一字符串都接受。
- 双板互测：每块板独立 alias，避免错乱。

> AI 通过 MCP 资源 `board://list` 来发现可用 alias；用 `board://{alias}` 查详情。

---

## 3. `BoardEntry` 字段

| 字段 | 必填 | 类型 | 默认 | 说明 |
|---|---|---|---|---|
| `soc` | ✓ | string | – | SoC 名（`RTL8735B`），用来在 `project_info.json5` 找烧录布局 |
| `port` | ✓ | string | – | `/dev/ttyUSB0` / `COM5` 等 |
| `memory_type` | – | `"nor"` / `"nand"` | `defaults.memory_type` | 板上 flash 类型 |
| `baudrate` | – | int | `defaults.baudrate` | 烧录与连接速率 |
| `monitor_baudrate` | – | int | `defaults.monitor_baudrate` | 串口监视速率 |
| `serial_log_record` | – | object | – | 串口日志落盘配置，见下文 |

> Pydantic schema 严格 `extra="forbid"`：未声明的字段一律拒绝。

---

## 3.1 `serial_log_record`：串口日志落盘

配置后，**只要通过 MCP 打开该板的串口（任意 serial 工具触发的 open）就开始记录日志，关闭串口即停止**。
日志捕获的是**完整串口流**——即使 agent 调用带 `drain_first` 的工具清掉了接收缓冲，**日志文件依然保留全部内容**。
每行带 `HH:MM:SS.mmm` 时间戳；多核 AAG 输出复用与 serial 工具相同的解析逻辑（`[HP]/[LP]/[AP]` 标签）。

| 字段 | 必填 | 类型 | 默认 | 说明 |
|---|---|---|---|---|
| `enable` | – | bool | `false` | 是否开启日志捕获。关闭时无任何后台线程，串口读路径与历史完全一致 |
| `log_dir` | – | string | `PROJECT_ROOT/mcp_serial_log` | 日志目录。相对路径挂在 `PROJECT_ROOT` 下；目录自动创建 |
| `file_name` | – | string | 自动生成 | 见下方命名/翻天规则 |

**文件名与翻天规则**（生成模式 `<alias>_<YYYYMMDD>_<HHMMSS>.log`，如 `RTL8735B_COM5_20260605_141930.log`）：

- `file_name` 为空 → 按当前系统时间生成，并**回写** `board_info.json5`。
- `file_name` 命中生成模式且日期==今天 → **复用**该文件（追加），不回写。
- `file_name` 命中生成模式但日期≠今天 → 生成新名并回写（保证**每天至少一个新日志**；长会话跨午夜也会自动滚动）。
- `file_name` **不**命中生成模式（用户自定义名）→ **原样使用，永不翻天、永不回写**。

> 注意：模式是**秒级**（`HHMMSS`，6 位）。形如 `..._1419.log`（分钟级 4 位）会被当成"用户自定义名"，不会按天滚动。
>
> 回写会按模板重渲染整个 `board_info.json5`（仅保留模板头注释，用户行内注释不被保留——与既有 `save_board_info` 行为一致）。

示例：

```json5
"RTL8735B_COM5": {
  "soc": "RTL8735B",
  "port": "COM5",
  "serial_log_record": {
    "enable": true
    /* log_dir 省略 → PROJECT_ROOT/mcp_serial_log；file_name 省略 → 首次打开自动生成并回写 */
  }
}
```

---

## 4. 本地板示例

```json5
"RTL8735B_COM5": {
  "soc": "RTL8735B",
  "port": "COM5"
  /* baudrate / monitor_baudrate 全部继承 defaults */
}
```

最小写法就是 `soc + port` —— 其它字段都有合理默认。

---

## 5. 双板互测示例

两块 Pro2 板同时连机器上：

```json5
"boards": {
  "RTL8735B_COM5": { "soc": "RTL8735B", "port": "COM5" },
  "RTL8735B_COM6": { "soc": "RTL8735B", "port": "COM6" }
}
```

测试用两个 alias，AI 可以同时 `serial_connect_tool("RTL8735B_COM5")` 与
`serial_connect_tool("RTL8735B_COM6")`，互不串扰（每个 alias 一份独立的 AAG 解析器与连接）。

---

## 6. `memory_type` 用法

- 取值 `"nor"` / `"nand"`，决定烧录时选用哪个 `.rdev` profile 与 `--memory-type`。
- 优先级：`boards.<alias>.memory_type` > `defaults.memory_type`（缺省 `"nor"`）。
- 例：nand 板写 `"memory_type": "nand"`，nor 板省略即可继承默认。

---

## 7. 注释规范

- 仅 `/* … */` 块注释。
- 不要使用行尾 `//`。
- 自动生成的 header 注释由 `templates.render_board_info` 维护，回写时会被重写。

---

## 8. 错误码与引导

| 错误码 | 触发 | 排查 |
|---|---|---|
| `BOARD_CONFIG_MISSING` | 文件不存在 | MCP 会自动建模板，按返回值里的 `template_path` 编辑 |
| `BOARD_CONFIG_PARSE_ERROR` | json5 语法错 | 检查行尾 `//`、尾随逗号、不平衡的 `{}` |
| `BOARD_CONFIG_INVALID` | schema 校验失败 | 按 `field_path` 定位字段；常见：缺 `soc`/`port` |
| `ALIAS_NOT_FOUND` | flash / serial 传了错的 alias | 错误信息里会列出已配置 alias；查 `board://list` |
| `PORT_NOT_FOUND` | 本机串口不存在 | 检查 USB 线 / dmesg / 改 `port` |
| `PORT_BUSY` | 端口被占 / 权限不足 | 关掉占用进程；Linux 检查用户是否在 `dialout` 组 |
| `PORT_OPEN_FAILED` | 其它本机打开失败 | 看 `message` |
| `FLASH_HW_ERROR` | 子进程烧录失败 | 检查 USB 线、板上电、boot mode；确认 `port` 与实际板匹配；看 `log_path` |
| `UARTFWBURN_NOT_FOUND` | Pro2 flash 工具缺失 | 确认 `tools/Pro2_PG_tool _v1.4.3/` 目录存在于 SDK |
| `CMAKE_NOT_FOUND` | cmake 不在 PATH | Windows：确认 msys64 路径正确；Linux：`sudo apt install cmake` |

---

## 9. 工作流（典型场景）

1. **首次使用**
   ```
   ai > 帮我烧 RTL8735B
   mcp > flash_firmware_tool("RTL8735B_COM5")
       ← BOARD_CONFIG_MISSING + template_path
   user > 编辑 board_info.json5，填入板子 alias
   ai > flash_firmware_tool("RTL8735B_COM5")
       ← success
   ```

2. **新增一块板**
   - 编辑 `boards`，加新 alias，AI 通过 `board://list` 自动发现。

3. **同型号多板互测**
   - 不同 alias 即可；AI 可同时 connect / read / write 互不冲突。

---

## 10. 与 `project_info.json5` 的关系

- `board_info.json5` 描述 **"哪块板"**（物理 / 链路）。
- [`project_info.json5`](project_info.md) 描述 **"烧什么"**（image / 地址）。
- `flash_firmware_tool(alias)`：alias → `board_info.json5`.boards[alias] →
  `board.soc` → `project_info.json5`.projects[soc] → image 列表 → 烧录子进程。
- 同一个 SoC 的 `ProjectEntry` 被多 alias 共享，是有意设计：换板不需要再写一份 layout。

---

## 11. 完整示例

```json5
/*
 * board_info.json5 — test-bench config.
 */
{
  "schema_version": 1.1,

  "defaults": {
    "baudrate": 115200,
    "monitor_baudrate": 115200,
    "memory_type": "nor"
  },

  "boards": {
    "RTL8735B_COM5": {
      "soc": "RTL8735B",
      "port": "COM5"
    },
    "RTL8735B_nand_COM6": {
      "soc": "RTL8735B",
      "port": "COM6",
      "memory_type": "nand"
    },
    "pro2_mini_ttyUSB0": {
      "soc": "RTL8735B",
      "port": "/dev/ttyUSB0",
      "serial_log_record": { "enable": true }
    }
  }
}
```
