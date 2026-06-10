# ET1 Track Cache 文件说明

`.et1trk` 是 ET1 tracking motion 在部署阶段使用的离线缓存文件。它从源
`.npz` motion 文件中抽取部署需要的数组，并以简单的二进制格式保存，方便
`State_Track::ReferenceLoader` 在运行时快速读取。

这份文档只说明 `.et1trk` 文件自身记录了什么信息，不包含具体 policy 的观测
layout、状态机切换或运行命令。

## 生成方式

如果 `motion_file` 指向 `.et1trk` 文件，部署代码会直接读取这个文件。

如果 `motion_file` 指向 `.npz` 文件，部署代码会调用转换脚本，在同目录生成
同名 `.et1trk` 文件：

```bash
python3 scripts/et1/convert_track_npz.py --input motion.npz --output motion.et1trk
```

转换脚本支持以下 profile：

- `auto`：要求基础 motion 数组；如果源文件中存在 foot-support/reference-COM
  数组，也会一起写入 cache。
- `latest_general_tracker`：要求基础 motion 数组和 GeneralTracker 额外数组都
  存在。
- `legacy`：只要求基础 motion 数组。

## 二进制结构

文件开头是固定 header：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| magic | `char[8]` | `ET1TRK1\0` |
| version | `uint32` | cache 格式版本，目前为 `1` |
| array_count | `uint32` | 文件内保存的数组数量 |

随后每个数组按以下结构依次保存：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| name_len | `uint32` | UTF-8 数组名的字节长度 |
| name | bytes | 数组名，例如 `joint_pos` |
| dtype_code | `uint32` | C++ loader 使用的数据类型编号 |
| ndim | `uint32` | 数组维度数量 |
| dims | `uint32[ndim]` | 数组 shape |
| byte_count | `uint64` | 数组数据区的字节数 |
| payload | bytes | C-order 连续存储的数组数据 |

支持的数据类型编号：

| Code | dtype |
| --- | --- |
| 1 | `float32` |
| 2 | `float64` |
| 3 | `bool` |
| 4 | `int32` |
| 5 | `int64` |
| 6 | `uint8` |
| 7 | `int8` |

## 基础数组

所有 ET1 tracking cache 都需要记录以下基础数组：

| 名称 | Shape | 含义 |
| --- | --- | --- |
| `joint_pos` | `(frames, 26)` | 参考关节位置 |
| `joint_vel` | `(frames, 26)` | 参考关节速度 |
| `body_pos_w` | `(frames, 27, 3)` | world frame 下的参考 body 位置 |
| `body_quat_w` | `(frames, 27, 4)` | world frame 下的参考 body 四元数，顺序为 `w, x, y, z` |
| `body_lin_vel_w` | `(frames, 27, 3)` | world frame 下的参考 body 线速度 |
| `body_ang_vel_w` | `(frames, 27, 3)` | world frame 下的参考 body 角速度 |

`body_quat_w[:, 0]` 会被当作 root body 的姿态。转换脚本会检查 root 四元数
是否归一化。

## 扩展数组

较新的 tracking 数据还会记录以下扩展数组：

| 名称 | Shape | 含义 |
| --- | --- | --- |
| `left_foot_contact_state` | `(frames,)` | 左脚支撑/接触状态编号 |
| `right_foot_contact_state` | `(frames,)` | 右脚支撑/接触状态编号 |
| `ref_com_rel_navi` | `(frames, 3)` | navigation frame 下的参考 COM 相对位置 |
| `ref_com_vel_navi` | `(frames, 3)` | navigation frame 下的参考 COM 速度 |

`left_foot_contact_state` 和 `right_foot_contact_state` 只能包含 `0`、`1`、
`2`。运行时会被转换成 6 维 `command_foot_support_state`：

```text
[left_state_one_hot_3, right_state_one_hot_3]
```

例如 `left=1`、`right=2` 时，结果为：

```text
[0, 1, 0, 0, 0, 1]
```

## 运行时读取

`State_Track::ReferenceLoader` 会把 `.et1trk` 中的数组读入逐帧序列。每次
control update 时，它根据当前时间选取对应帧，并从这些序列中更新参考关节、
root 姿态、速度、脚支撑状态和参考 COM 信息。

如果 cache 中没有 `left_foot_contact_state` 或 `right_foot_contact_state`，
`command_foot_support_state` 会保持全 0。`ref_com_rel_navi` 和
`ref_com_vel_navi` 缺失时，对应参考 COM 观测也会保持全 0。
