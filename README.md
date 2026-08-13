# CUArm 数据可视化

## 运行

```bash
cd cuarm_data_visualizer
python3 -m http.server 8765
```

浏览器打开 http://127.0.0.1:8765

## 字段从哪来？

日志列布局以 `cuarm_rt_control/app/log_utils.ipp` 的 `writeToLogFile` 为准。

可视化侧对应文件：

| 文件 | 作用 |
|------|------|
| `log_schema.json` | 字段布局定义（改 C++ 写日志后同步改这个） |
| `log_*.structure.json` | 运行时由控制程序自动生成，含关节数 + 完整列名 |

## 推荐用法

1. 打开 `log_YYYYMMDD_HHMMSS.txt`
2. 再打开同名的 `log_YYYYMMDD_HHMMSS.structure.json`（「结构 JSON」按钮）——列名与维度会自动对齐
3. 若没有 sidecar：在左侧填关节/夹爪数（本例日志为 `6` 关节、夹爪空 → **155** 列），靠 `log_schema.json` 解析

## 改字段后怎么同步（一劳永逸）

改 `writeToLogFile` 时，同步三处即可：

1. `log_utils.ipp` 里的 `writeToLogFile`（写数据）
2. 同文件的 `appendLogColumnNames`（写 sidecar 列名，顺序必须一致）
3. `cuarm_data_visualizer/log_schema.json`（网页离线/旧日志解析）

重新编译运行控制程序后，新日志会自带 `.structure.json`，网页优先用它，以后不易再对不上。
