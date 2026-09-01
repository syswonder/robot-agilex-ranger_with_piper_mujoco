# 架构说明

## 环境与机器人解耦

环境由 `assets/environments/manifest.json` 注册。每个条目声明视觉模式、MJCF、
运行资产索引、出生点和可选相机预设。`SceneManager` 将选中的环境资产与机器
人包复制到同一个 MuJoCo MEMFS 场景目录，再通过 `<include>` 组合模型。

机器人包不引用 Kitchen、Meeting Room、SceneSmith 或任何外部绝对路径。环境也不包含
Ranger/Piper，因此后续可以独立增加机器人包或场景包。

## 两类视觉流程

`visualMode=spz` 时，MJCF 只加载不可见的 group 3 环境碰撞；Spark 加载 SPZ
并应用离线变换。`visualMode=mesh` 时，MJCF 同时加载 group 1 纹理视觉 mesh
和 group 3 静态碰撞。切换模式会销毁旧 MuJoCo 模型、重建控制器和传感器，
并同步启停 Spark。

## 传感器层

Ranger/Piper 的传感器位姿来自 MuJoCo site/camera：

- MID-360：4096 ray，360 度水平、-7 至 52 度垂直视场；
- Gemini 336L：320x240 RGBA，160x120 几何深度；
- Piper 腕部相机：320x180 RGBA；
- IMU：MuJoCo 原生 gyro 和 accelerometer。

LiDAR 与几何深度通过 `mj_multiRay` 查询 group 3。RGBA 由同一个 Three.js
场景离屏渲染，因此 SPZ 和 mesh 两种视觉都能进入彩色相机画面。

## SceneSmith 静态化

预处理读取官方 MuJoCo XML 和 OBJ 顶点，在每个顶层家具自身坐标系内计算
AABB，保留顶层姿态后形成定向包围盒。自由关节与惯性节点被移除，所有家具
成为固定 body。二维出生点搜索在地板边界内对障碍 AABB 做距离场采样，并用
`--robot-radius` 控制安全半径。

卧室 `005`、客厅 `030`、餐厅 `036` 和办公室 `125` 共用同一生成器，但各自
保存运行资产索引和出生点。源 tar 与解压目录只用于离线重建，不是
浏览器运行依赖。

生成器会按内容合并重复的 mesh、材质和纹理，并重新索引 OBJ 中重复的顶点与
UV。这些转换不减少三角面或纹理分辨率。浏览器并发复制环境资产到 MEMFS，
Three.js 侧按 MuJoCo 材质和纹理 ID 复用 GPU 资源；本地服务允许 OBJ、PNG、
SPZ、WASM 等大型静态资产进入浏览器缓存。

这种表示优先保证导航、避障、LiDAR/RGB-D 和实时步进。它不声称能替代精细
操作接触模型；需要交互的少数家具应作为独立 MJCF 对象包加入，而不是把整个
房间重新变成动态刚体集合。
