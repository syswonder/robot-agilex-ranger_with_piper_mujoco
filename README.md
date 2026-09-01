# MuJoCo Robonix

一个可直接在浏览器中运行的 MuJoCo 移动操作机器人仿真项目。项目只包含
`Ranger Mini V3 + Piper` 本体，并支持两种由环境自动选择的视觉方式：

- SPZ Gaussian Splatting：真实感视觉与独立 MuJoCo 碰撞模型；
- 带纹理 Mesh：视觉 mesh、静态家具碰撞、LiDAR 和 RGB-D 使用同一个坐标系。

项目包含运行所需的场景和机器人资产。克隆后不需要下载 SceneSmith 原始数据、
MuJoCo 源码或其他模型仓库。

## 已有场景

| 环境 ID | 页面名称 | 视觉 | 内容 |
| --- | --- | --- | --- |
| `kitchen` | Kitchen - SPZ | SPZ | 厨房 |
| `meeting_room` | Meeting Room - SPZ | SPZ | 会议室 |
| `scenesmith_room_005` | SceneSmith Bedroom 005 - Mesh | Mesh | 卧室 |
| `scenesmith_room_030` | SceneSmith Living Room 030 - Mesh | Mesh | 客厅 |
| `scenesmith_room_036` | SceneSmith Dining Room 036 - Mesh | Mesh | 餐厅 |
| `scenesmith_room_125` | SceneSmith Office 125 - Mesh | Mesh | 办公室 |

页面根据所选环境自动启用 SPZ 或 Mesh，不提供单独的 `visual_mode` 选项。

## 系统要求

- Linux、macOS 或 Windows；
- Node.js 18 或更高版本；
- Python 3；
- 支持 WebGL 2 的现代 Chrome、Chromium 或 Edge。

SPZ 和 Mesh 都在浏览器本地渲染。建议使用独立显卡；Meeting Room 包含约
479 万个原始 splat，第一次进入时需要等待下载和 LOD 构建。

## 启动

```bash
git clone <your-repository-url>
cd mujoco_robonix
npm ci
npm start
```

浏览器打开：

```text
http://localhost:5180/
```

也可以通过 URL 直接打开场景：

```text
http://localhost:5180/?environment=meeting_room
http://localhost:5180/?environment=scenesmith_room_036
```

端口被占用时可绕过 npm 脚本指定其他端口：

```bash
python3 scripts/serve.py --bind 0.0.0.0 --port 8080
```

必须通过 HTTP 服务打开，不能直接双击 `index.html`。项目在运行时从
`node_modules` 加载 MuJoCo WASM、Three.js 和 Spark。

## 操作

| 功能 | 按键 |
| --- | --- |
| 前进 / 后退 | `W` / `S` |
| 左右转向 | `A` / `D` |
| 蟹行 | `Q` / `E` / `Z` / `C` |
| 原地旋转 | `J` / `L` |
| 急停 | `Space` |
| 重置机器人 | `X` |
| Piper 关节 1-6 | `1/Y`、`2/U`、`3/I`、`4/O`、`5/P`、`6/[` |
| 夹爪开合 | `V` / `B` |
| 重置观察相机 | `Ctrl+A` |

鼠标拖动空白处或场景用于旋转视角，只有机器人本体允许拖动。右上角 GUI
可以切换环境、暂停仿真、控制执行器以及读取传感器。

## 传感器

机器人包含：

- Livox MID-360 风格 3D LiDAR；
- MID-360 IMU；
- 前置 Orbbec Gemini 336L RGB-D；
- Piper 腕部 RGB 相机。

SPZ 场景的彩色图像来自 Gaussian 视觉，深度与 LiDAR 来自 MuJoCo 碰撞模型；
Mesh 场景的三类数据均使用 mesh 场景对应的物理空间。环境碰撞统一使用
`geom group="3"`，机器人碰撞使用 group 4，避免传感器返回本体反射。

浏览器控制台或上层应用可以调用：

```js
window.mujocoSensors.list();
window.mujocoSensors.scanLidar();
window.mujocoSensors.readImu();
window.mujocoSensors.captureCamera('front_rgbd');
window.mujocoSensors.previewCamera('wrist_rgb');
window.mujocoSensors.startLiveCamera('front_rgbd');
window.mujocoSensors.stopLiveCamera();
window.mujocoSensors.showLidar(true);
```

## 添加 SPZ 场景

每个 SPZ 环境是一个独立目录。最小运行包如下：

```text
assets/environments/my_room/
  scene.spz
  collision.xml
  transform.json
  spawn.json
```

要求：

- `scene.spz` 是 Spark 可以读取的视觉文件；
- `collision.xml` 是完整的 MuJoCo XML，必须包含地面和障碍物，传感器可见的
  环境 geom 使用 `group="3"`；
- `transform.json` 把 SPZ 的原始坐标映射到 MuJoCo/Three.js 世界；
- `spawn.json` 给出无碰撞的机器人平面出生点和朝向。

`transform.json` 的最小格式：

```json
{
  "schemaVersion": 1,
  "sceneId": "my_room",
  "transform": {
    "rotationQuaternion": [0, 0, 0, 1],
    "scale": 1,
    "position": [0, 0, 0],
    "groundY": 0
  }
}
```

`spawn.json` 的最小格式：

```json
{
  "schemaVersion": 1,
  "sceneId": "my_room",
  "position": [0, 0],
  "yaw": 0
}
```

最后向 `assets/environments/manifest.json` 的 `environments` 数组加入：

```json
{
  "id": "my_room",
  "label": "My Room - SPZ",
  "visualMode": "spz",
  "xmlPath": "./assets/environments/my_room/collision.xml",
  "spzPath": "./assets/environments/my_room/scene.spz",
  "transformPath": "./assets/environments/my_room/transform.json",
  "spawnPath": "./assets/environments/my_room/spawn.json",
  "camera": null
}
```

刷新页面后场景会自动出现在 `Environment` 列表。原始 PLY、转换日志、碰撞盒
诊断图等中间文件已被 `.gitignore` 排除，不应放入 Git；运行目录只保留上述
四个文件。

## 添加 Mesh 场景

通用 Mesh 环境最少需要：

```text
assets/environments/my_mesh_room/
  scene.xml
  index.json
  spawn.json
  meshes/...
```

`scene.xml` 是环境 MJCF，不要包含机器人；`index.json` 是需要复制到 MuJoCo
浏览器 MEMFS 的相对资产路径数组；`spawn.json` 与 SPZ 场景相同。manifest
条目格式：

```json
{
  "id": "my_mesh_room",
  "label": "My Mesh Room - Mesh",
  "visualMode": "mesh",
  "xmlPath": "./assets/environments/my_mesh_room/scene.xml",
  "filesPath": "./assets/environments/my_mesh_room/index.json",
  "spawnPath": "./assets/environments/my_mesh_room/spawn.json",
  "camera": {
    "position": [4, 3, -4],
    "target": [0, 0.7, 0]
  }
}
```

针对 SceneSmith 的原始 MuJoCo 包，可以使用项目保留的标准库脚本生成静态、
裁剪后的运行包：

```bash
python3 scripts/prepare-scenesmith.py \
  --source third_party/scenesmith/source/scene_036/mujoco \
  --output assets/environments/scenesmith_room_036 \
  --scene-id scenesmith_room_036 \
  --source-archive scene_036.tar \
  --robot-radius 0.45
```

`third_party/` 整体不会上传。重新生成时自行把 SceneSmith 原始目录放到该位置；
脚本会静态化家具、生成碰撞盒、搜索出生点、去除未引用资产并生成
`index.json`。生成后的 `scene.xml`、`spawn.json`、OBJ 和 PNG 才需要提交。

## 项目结构

```text
index.html                                  浏览器入口
src/                                        仿真、渲染、交互和传感器代码
assets/environments/manifest.json           环境注册表
assets/environments/kitchen/                Kitchen SPZ 运行包
assets/environments/meeting_room/           Meeting Room SPZ 运行包
assets/environments/scenesmith_room_*/       SceneSmith Mesh 运行包
assets/robots/ranger_mini_v3_piper/          自包含机器人包
scripts/serve.py                             静态文件服务
scripts/prepare-scenesmith.py                SceneSmith 运行包生成器
docs/ARCHITECTURE.md                         架构与坐标层说明
```

仓库运行资产约 285 MB，最大单文件为约 53 MB 的 `meeting_room.spz`，低于
GitHub 100 MB 单文件限制。`node_modules`、构建目录、测试、数据集原件和所有
可再生报告均不会上传。

## 借鉴与上游项目

- [MuJoCo-GS-Web](https://github.com/Vector-Wangel/MuJoCo-GS-Web)：前端交互、
  MuJoCo 与 Gaussian Splatting 同场景渲染基础；
- [mujoco-wasm](https://github.com/zalo/mujoco_wasm) / `mujoco-js`：浏览器中的
  MuJoCo WebAssembly 运行时；
- [Spark](https://github.com/sparkjsdev/spark)：SPZ / Gaussian Splatting 渲染；
- [SceneSmith](https://github.com/nepfaff/scenesmith) 和
  [示例场景数据集](https://huggingface.co/datasets/nepfaff/scenesmith-example-scenes)：
  四个 Mesh 室内场景；
- [MuJoCo Menagerie AgileX Piper](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper)：
  Piper MJCF 与 mesh；
- [Robonix](https://github.com/syswonder/robonix)：Ranger-Piper 实机布局与传感器
  位姿参考；
- [Piper_mujoco](https://github.com/soulde/Piper_mujoco)：Piper 控制参考；
- [MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR)：MuJoCo 批量射线
  LiDAR 实现参考。

根目录代码使用 ISC License。SceneSmith 派生场景的说明位于各场景的
`NOTICE.md`；Piper 的 MIT License 副本位于
`assets/robots/ranger_mini_v3_piper/third_party/`。使用或再分发时请同时遵守
各上游资产的许可证。
