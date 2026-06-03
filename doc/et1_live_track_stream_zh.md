# ET1 实时 Track Reference 流协议

ET1 `State_Track` 现在支持两种 reference backend：

- 文件模式：继续读取 `.et1trk` 或由 `.npz` 自动转换。
- 实时模式：通过 ZMQ `SUB` 接收外部发布的 reference motion，控制端仍按自己的 `step_dt = 0.02` 也就是 50Hz 消费。

注意：这个接口发布的是 **reference motion**，不是 `lowcmd`，也不是 policy action。deploy 端收到 reference 后，仍然由 `State_Track` 组 observation，并运行 `GeneralTrackerCJM` 或 `GeneralTrackerCLN` policy 来输出控制。

## 配置

在 `deploy/robots/et1/config/config.yaml` 对应 Track 状态下打开：

```yaml
live_stream:
  enabled: true
  endpoint: tcp://127.0.0.1:5557
  topic: et1_track
  max_queue_frames: 2000
```

当前 `GeneralTrackerCJM` 和 `GeneralTrackerCLN` 都支持这套配置。发布端推荐：

```text
ZMQ PUB bind: tcp://*:5557
deploy SUB connect: tcp://127.0.0.1:5557
topic: et1_track
rate: 50Hz
```

`Velocity` 状态会常驻监听同一 ZMQ topic。收到合法的 `ET1LIVE1` reset 首帧后，控制器自动从 `Velocity` 切换到配置的 tracker 状态；没有新轨迹时继续保持 `Velocity`：

```yaml
Velocity:
  live_stream_trigger:
    enabled: true
    endpoint: tcp://127.0.0.1:5557
    topic: et1_track
    target_state: GeneralTrackerCJM
```

将 `target_state` 改成 `GeneralTrackerCLN` 即可自动切换到 CLN tracker。键盘 `3` 和 `9` 仍保留为手工调试入口，但正常流式链路不再要求手工切换状态。

启动预缓冲不需要手写配置。控制端会根据 policy deploy 配置自动推断：如果 observation 使用 `future_commands`，默认需要未来 25 帧，因此执行 frame 0 前会等待队列里已有 frame 0..25；如果 `future_commands.params.horizon` 明确配置了其他值，则按该值等待 `horizon + 1` 帧；如果没有使用 `future_commands`，只等待当前帧。发布频率高于控制端消费频率时，控制端会按 policy deploy yaml 的 `step_dt` 节奏从队列取帧；队列超过 `max_queue_frames` 时丢弃最老帧。

## ZMQ 消息

推荐发布 multipart：

1. topic 字符串，例如 `et1_track`
2. 二进制 payload

payload = little-endian header + float32 数组。

Header C 结构：

```cpp
struct LiveWireHeader {
    char magic[8];          // "ET1LIVE1"
    uint32_t version;       // 1
    uint32_t flags;         // bit0 reset, bit1 end
    uint64_t sequence;      // 单调递增
    uint64_t publish_time_ns;
    uint32_t float_count;
    uint32_t reserved;
};
```

float32 数组顺序：

```text
joint_pos[26]              # policy 26 关节顺序，不是 SDK slot
joint_vel[26]              # policy 26 关节顺序
root_quat_wxyz[4]          # world root quaternion, wxyz，发布前应归一化
root_lin_vel_w[3]          # world root linear velocity
root_ang_vel_w[3]          # world root angular velocity
left_foot_contact_state    # 可选，0=flat contact, 1=toe contact, 2=airborne
right_foot_contact_state   # 可选，0=flat contact, 1=toe contact, 2=airborne
ref_com_rel_navi[3]        # 可选，reference COM 相对 root 的导航系表达
ref_com_vel_navi[3]        # 可选，reference COM velocity 的导航系表达
```

最少需要前 62 个 float。完整推荐 payload 是 70 个 float。可选字段没有发布时控制端会补零或 unknown foot state。

发送要求：

- `sequence` 单调递增。
- 新轨迹首帧 `flags |= 1`，表示 reset。
- 发布频率建议和 deploy `step_dt: 0.02` 对齐，即 50Hz。
- deploy 端当前断流后会 hold 最后一帧；真机前建议加上层 watchdog 或停机保护。

## GMR 流式发布端

`gx_loco_deploy` 的 GMR 子模块提供了 BVH -> ET1 reference 的流式发布脚本：

```bash
/home/galbot/miniconda3/envs/gmr/bin/python \
  thirdparty/GMR-galbot/scripts/retarget_bvh_et1_stream.py \
  /path/to/motion.bvh \
  --publish_live \
  --no-run_scaletrack_policy
```

`--live_bind` 和 `--live_topic` 已经默认是 deploy 需要的值：

```text
--live_bind  tcp://*:5557
--live_topic et1_track
```

因此通常不需要手动指定。发布端会在上游产出新的可用 50Hz reference frame 后立刻打包 ZMQ payload，首帧带 reset flag，后续 sequence 递增。

`--no-run_scaletrack_policy` 表示不在 GMR 端运行本地 ScaleTrack rollout。此时发布端将原始 50Hz KPTS 作为 reference motion 流式发送给 deploy。它不会关闭 ZMQ 发布。

如果保留默认 `--run_scaletrack_policy`，ScaleTrack 作为在线洗数据工具存在。脚本先将原始 KPTS 喂给本地 ScaleTrack rollout；每当 rollout 产出新的状态帧，就立即将洗过的 `joint_pos`、`joint_vel`、root 状态、脚接触状态和 COM 信息重新包装为 reference motion，通过 ZMQ 发送给 deploy。ScaleTrack 需要未来 reference 帧，因此该模式带有固定延迟。

两种模式下，deploy 收到的都仍然是 reference motion。deploy 端继续运行 `GeneralTrackerCJM` 或 `GeneralTrackerCLN` tracker policy，再输出实际控制命令。ZMQ 不发送本地 ScaleTrack 的原始 ONNX action，也不直接发送 `lowcmd`。

默认推荐保留 `--run_scaletrack_policy`，使用洗过的 reference motion。需要绕过清洗阶段做对照测试时，再显式添加 `--no-run_scaletrack_policy`。

## 历史/未来帧预留

当前 policy 的历史 observation 仍由 `observation_manager` 自己维护；实时流只替代 reference command。需要未来帧的 `future_commands` 会从实时队列 lookahead 读取。对于未来 N 帧的 policy，控制端应在启动前等待 `N + 1` 帧，让执行当前帧时队列中已经保留完整未来窗口；当前代码会对 `future_commands` 自动推断这个等待帧数。发布端持续快于消费端后，后续 tick 通常不再需要等待。

## 与 policy 绑定的参数

- 控制消费频率来自 policy deploy yaml 的 `step_dt`，不再在 FSM 配置里单独写 `fps`。
- 未来帧等待数来自 `future_commands`，默认 horizon 为 25；可用 `future_commands.params.horizon` 覆盖。
- `max_queue_frames`、`endpoint`、`topic`、ZMQ high-water mark 属于传输层参数，不由 policy 推断。
