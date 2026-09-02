# Robonix 接入设计

## 1. 边界与拆包

项目按“一个本体部署、六个硬件边界 primitive”组织，而不是为每个场景复制一
套本体包。

| Provider | 物理边界 | 主要能力 |
| --- | --- | --- |
| `ranger_chassis` | Ranger 底盘 | move、odom |
| `mid360_lidar` | MID-360 | 2D scan、3D point cloud |
| `mid360_imu` | 雷达内置 IMU | angular velocity、linear acceleration |
| `front_camera` | 前置 Gemini RGB-D | RGB、depth、intrinsics/extrinsics |
| `wrist_camera` | Piper 腕部 RGB-D | RGB、depth、intrinsics |
| `piper_ctl` | Piper 与夹爪 | joint state/command、TCP pose/command |

这种粒度与 Robonix provider 生命周期及真实硬件故障边界一致：单个相机重启不
需要重启底盘，mapping/Nav2 可以通过 provider ID 依赖传感器。`soma.yaml` 再把
这些 provider 组装成一个 `mobile_manipulator`。

所有本地包优先读取 `rbnx boot` 注入的 `RBNX_INSTANCE_NAME`，所以同一包可以在
部署清单中安全重命名或创建多个实例。启动包装器中的默认名称只用于单包
`rbnx start` 调试，不是运行时身份配置。

## 2. 数据与控制链路

浏览器是 MuJoCo 真值和物理步进的唯一所有者。`MujocoBridgeClient` 以固定频率
发送机器人状态、任务物体、LiDAR、IMU 和相机帧；Python bridge 转为 ROS 2
message 并维护 TF。反向的 `/cmd_vel`、Piper 关节/TCP、抓取和 reset 命令经同一
WebSocket 返回浏览器。primitive 只通过 ROS 2 接口访问仿真，不读取浏览器内部
状态。

关键 topic：

| Topic | 类型 | 方向 |
| --- | --- | --- |
| `/odom`, `/tf` | Odometry / TF | 仿真 -> Robonix |
| `/scan`, `/mid360/points`, `/mid360/imu` | sensor_msgs | 仿真 -> Robonix |
| `/front/{rgb,depth,camera_info}` | sensor_msgs | 仿真 -> Robonix |
| `/wrist/{rgb,depth,camera_info}` | sensor_msgs | 仿真 -> Robonix |
| `/arm/joint_states_single`, `/arm/end_pose` | JointState / PoseStamped | 仿真 -> Robonix |
| `/cmd_vel` | Twist | Robonix -> 仿真 |
| `/arm/joint_command`, `/arm/pos_command` | JointState / PoseStamped | Robonix -> 仿真 |
| `/sim/pick_command`, `/sim/reset` | String / Empty | Robonix -> 仿真 |
| `/sim/task_objects`, `/sim/pick_status` | JSON String | 仿真 -> Robonix |

所有 ROS 2 容器统一使用 Fast DDS 和 `/clock` 的仿真时间，避免 mapping/Nav2 与
本地 primitive 的 discovery 分区。

## 3. 导航、避障与建图

RTAB-Map 消费底盘 odom、MID-360 和前置 RGB-D，输出占据栅格与 map-frame pose。
Nav2 消费该栅格、`/scan` 和 odom；局部/全局 costmap 的机器人 footprint 与
`soma.yaml` 中 Ranger 外形一致。所有场景碰撞 geom 对射线可见，因此 SPZ 只
负责外观，激光、深度和接触都使用与其配准的 MuJoCo collision 模型。Mesh 场景
直接从同一几何空间产生视觉与传感器返回。

导航命令由 `rbnx chat` 规划到 `robonix/service/navigation/navigate`，而不是通过
网页键盘。Nav2 控制器发布 `/cmd_vel`，bridge 将速度传给 Ranger 全向驱动器。
本体包将到达容差设为 `0.04 m` 和 `0.05 rad`；这是后置 Piper 操作位需要的精度，
可避免普通移动导航的宽松到达判定让目标落到机械臂工作区之外。
全局 costmap 使用固定 `16 m x 12 m` 滚动窗口，在叠加实时 RTAB-Map 的同时允许
规划到当前传感器覆盖范围之外的未知室内区域，避免动态 SLAM 地图初始边界过小而
拒绝合法目标。Navigation 安全守卫的终端区设为 `0.02 m`，小于 Nav2 的位置
容差，使其在 DWB 完成路径末端位置对齐后再监督最终朝向，避免将正常的末端路径
跟随误判为原地旋转无进展。

## 4. 抓取

`pick` skill 遵循 Robonix 标准 pick contract，并补充 `put_down`：

1. 从 `/sim/task_objects` 获取可操作对象和位置；
2. 获取腕部 RGB 图像；
3. 使用配置的 OpenAI-compatible VLM 将自然语言指代约束到允许对象 ID；
4. 向 `/sim/pick_command` 发送目标；
5. 浏览器控制器执行预抓取、下探、闭合、抬升和持续持稳状态机；
6. 只有夹爪接触目标且抬升后持续稳定 3 秒才返回抓取成功；
7. `put_down` 将物体降到原支撑高度，松爪并确认稳定，再后撤和收纳机械臂。

这套实现用于验证 Robonix 任务链与移动操作闭环，不宣称是任意物体的通用抓取
规划器。当前可操作物来自所选环境 `objects.xml` 的白名单；静态 SceneSmith 家具
不可抓。动态物体属于环境包，不随机器人出生位姿移动。

## 5. 场景与本体的关系

本体 package 不应包含某个房间的 SPZ 或 mesh 生成逻辑。环境注册表选择：

- 视觉资产（`scene.spz` 或 mesh）；
- 物理场景 XML；
- 两者的坐标变换；
- 机器人出生位姿；
- 可选的环境级 `objectsPath` 及动态任务物体世界位姿。

本体、sensor pose、topic 和 Robonix capability 在切换环境时保持不变。新增场景
只需制作运行资产并添加 manifest 条目，service/primitive 无需复制。

PLY -> SPZ 和 PLY -> collision 是离线“环境编译”链路。其最终产物进入
`assets/environments/<id>/`；原始 PLY、模型缓存、诊断图和转换报告不属于运行
仓库。碰撞模型必须单独验证地面连续性、机器人出生点和射线可见 group，视觉
质量不能作为碰撞正确性的依据。

## 6. 生命周期与启动

`sim/start.sh` 只启动 ROS bridge、HTTP 服务和浏览器，并在前台维持仿真生命
周期。浏览器连接 bridge 后，操作者在第二个终端从仓库根目录直接执行
`rbnx boot`；primitive/service/skill 的启动完全由部署清单管理。停止时先用
`rbnx shutdown` 回收 provider 和 service，再用 `sim/stop.sh` 关闭仿真。两层
互不代管，行为与 Robonix Webots example 一致。

`scripts/acceptance.sh` 是发布门槛：要求所有 provider ACTIVE、传感器有效、地图
同时含 free/occupied cell、Nav2 lifecycle active、底盘产生实际里程、通过 Nav2
到达操作位，以及水杯持续持稳、安全放置和机械臂归位。它不是仅检查 topic 存在，
而是验证物理效果。

当前已测试的 Robonix 0.1 运行时仍通过各命名空间的 `*/driver` 生成生命周期
服务，这也与当前真实 Ranger 包一致。新版 Book 已迁移到共享
`robonix/lifecycle/driver`；升级 Robonix API 和代码生成器时，应按该版本迁移
指南同时删除清单 Driver 条目和本地 Driver contract，不能只改清单。

## 7. 凭据与可复现性

VLM key 只从项目 `.env` 或当前 shell 注入环境，`.env` 被忽略，部署
清单只引用变量。仓库保存 npm lockfile、Dockerfile、primitive/skill 源码和全部
运行场景资产；首次构建还需要外部 Robonix 源码及官方 map/navigation/explore
服务包，版本更新应先在验收脚本中重新验证。
