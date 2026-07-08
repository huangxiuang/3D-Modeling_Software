# Known Issues

记录当前项目中已知的 Bug、待修复问题、以及待实现的功能缺口。

---

# notes：
@sisyphus，每次修改完，记得更新这个.md文件

## Issue 模板

```markdown

### [ID-0] 模版

- **状态**: 待修复 / 修复中 / 已验证修复
- **模块**: [关联功能模块]
- **严重程度**: 🔴 严重 / 🟡 中等 / 🟢 轻微     
- **发现日期**: YYYY-MM-DD
**问题描述**:
清晰描述问题现象。
模版


### [ID-1] 添加飞机姿态参数 ✅

- **状态**: 已验证修复
- **模块**: 对象控制中的所有aircraft
- **严重程度**: 🟡 中等 
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29

**问题描述**:
添加飞机姿态：比如 航向角（通常以正北方向为0°，报数0-360，90对应东，180对应南，270对应西）、俯仰角、滚转角

**预期行为**:
跟其他参数一样用滑块控制。

以下为具体：

1. 航向角（偏航角，Yaw）—— 0° 到 360°

定义：机头在水平面上的投影与正北方向的夹角，顺时针为正。
范围：0° ~ 360°（也有用 -180° ~ 180° 表示的）。
注意：0° = 360°，都表示正北。当角度超过360°（比如持续右转）时，工程上通常会通过取余运算（mod 360）将其映射回这个区间。
2. 俯仰角（Pitch）—— -90° 到 +90°

定义：机身纵轴（机头方向）与水平面的夹角。
范围：-90° ~ +90°。

+90°：机头垂直向上（笔直爬升）。
0°：机身水平。
-90°：机头垂直向下（笔直俯冲）。
物理限制：飞机不可能越过 ±90° 飞向"另一侧"（那会导致机身倒扣），且这个范围完美避开了我之前提到的"万向节锁"发生临界点。
3. 滚转角（Roll）—— -180° 到 +180°（或 -90° ~ +90° 工程简化）

定义：绕机身纵轴旋转的角度，即机翼相对于水平面的倾斜程度。
标准定义范围：-180° ~ +180°（全范围定义）。

0°：机翼水平。
+90°：右侧机翼向下，左侧向上（右坡度）。
-90°：左侧机翼向下（左坡度）。
±180°：机身完全倒扣（机腹朝上）。
工程简化：在常规载人客机或一般飞行场景中，为了避免歧义，飞控系统通常会将滚转角限制在 -90° ~ +90° 之间（因为民航客机不会翻转倒飞）。

**修复内容**:
- 在`scene_builder.py`中移除了飞机模型的`rotate_y(-5)`烘焙旋转，使旋转完全由UI控制
- 在`main_window.py`的"对象控制"面板中添加了航向角(Yaw: 0-360°)、俯仰角(Pitch: -90-90°)、滚转角(Roll: -180-180°)三个滑块
- 修改`_apply_obj_transform_to_actor()`以支持VTK旋转变换 (Z-Y-X欧拉角顺序)
- 更新`_on_obj_select_changed()`和`_set_slider_value()`以正确加载和显示旋转值
- 旋转数据与位置/缩放一起持久化到JSON场景文件中

---
### [ID-2] 删除河流透明度 ✅

- **状态**: 已验证修复
- **模块**: 显示参数
- **严重程度**: 🟢 轻微 
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29

**问题描述**:
删除河流透明度参数，这个跟我建模本意无关

**修复内容**:
- 从`config.py`的`DEFAULT_CONFIG`中删除`river_opacity`配置项
- 从`main_window.py`的`_setup_docks()`中删除河流透明度滑块
- 删除`_on_river_opacity()`回调方法

### [ID-3] 测距，测角，添加路径错误 再次修改

- **状态**: 已验证修复
- **模块**: 路径规划 / 测量
- **严重程度**: 🔴 严重  
- **发现日期**: 2026-06-29
- **修复日期**: 2026-07-03

**问题描述**:
路径生成的位置不是我鼠标点的位置

**根因分析**:
Qt事件系统的坐标系原点在**左上角**，而VTK显示坐标系的原点在**左下角**。先前代码将Qt的Y坐标原封不动传递给`vtkPropPicker`和`vtkWorldPointPicker`，导致拾取射线在Y轴方向完全翻转，点击位置与实际拾取位置严重偏离。当PropPicker因Y翻转未命中任何物体时，回退到WorldPointPicker返回焦平面深度值，坐标误差进一步放大。

**修复内容**:
- 在`ClickablePlotter`中添加`_vtk_display_y()`方法，将Qt Y坐标转换为VTK显示Y坐标 (height - y)
- 修复`_process_click()`中vtkPropPicker和vtkWorldPointPicker的Y坐标
- 修复`_on_3d_move()`中鼠标悬停坐标拾取的Y坐标
- 修复`_get_name_from_pick()`中VTK picker的Y坐标
- 在`_set_coord_system()`中同步更新坐标输入spinbox的轴标签前缀，防止用户在非ENU坐标系下混淆坐标轴含义

**2026-07-03 再次升级**:
将_click处理中的`vtkPropPicker`升级为`vtkCellPicker`（`SetTolerance=0.001`），提供逐cell级别的精确表面交叉计算，避免多覆盖层叠时PropPicker的prop级粗粒度拾取误差。PropPicker作为备选回退，最终保底使用WorldPointPicker。同时三维场景中terrain已永久可见（`visible=True`），PropPicker不再因地形不可见而失败。


### [ID-4] 把鼠标坐标放在相机坐标前面 ✅

- **状态**: 已验证修复
- **模块**: 坐标信息 (坐标信息Dock)
- **严重程度**:  🟡 中等    
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29
**问题描述**:鼠标坐标放在相机坐标前面

**修复内容**:
- 在`_update_info()`中将鼠标世界坐标的显示调整到相机信息之前

### [ID-5] 只有选中aircraft的时候才有航向角俯仰角和滚转角 ✅

- **状态**: 已验证修复
- **模块**: 对象控制 (Dock面板)
- **严重程度**: 🔴 严重 
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29
**问题描述**: 只有选中aircraft的时候才有航向角俯仰角和滚转角，目前所有对象都有这个参数，但是我要求只有选中aircraft的时候才会有这个参数的数值。

**修复内容**:
- 在`_on_obj_select_changed()`中添加`is_aircraft = "aircraft" in clean_name.lower()`判断
- 将航向角/俯仰角/滚转角三个滑块放入`self._attitude_container`容器中
- 非aircraft对象时`self._attitude_container.setVisible(False)`隐藏

### [ID-6] 航向角俯仰角和滚转角不对 ✅

- **状态**: 已验证修复
- **模块**: 对象控制 (aircraft transform)
- **严重程度**: 🔴 非常严重    
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29

**问题描述**:
现在的航向角俯仰角和滚转角以坐标系原点为轴旋转这是不对的，我旋转它时连坐标都变了，我需要的是aircraft局部旋转（Local Rotation）

绕飞机自身的轴旋转
轴的方向随飞机姿态变化
因为你在建模的时候是把飞机作为很多部件组合起来的，所以你需要非常认真仔细的确认哪些需要局部旋转，比如机身，机翼等，最终呈现的效果应该得是合理的，不能机身转了机翼没转，重新修改

可以参考opengl的旋转矩阵，来实现局部旋转。（你也可以不参考，你自己决定）

**根因分析**:
`_apply_obj_transform_to_actor()`依赖VTK的`PreMultiply`/`PostMultiply`矩阵级联模式来构建变换。
虽然数学上等价，但VTK版本间级联行为不稳定，导致旋转中心偏移到世界原点而非物体自身中心。
简单切换PreMultiply/PostMultiply无法根治——VTK的级联模式在不同版本和环境下行为不一致。

**修复内容**:
- **彻底移除VTK矩阵级联**，改用numpy显式构建4×4齐次变换矩阵
- 直接实现变换公式：`p' = offset + Rz(yaw)·Ry(pitch)·Rx(roll)·scale·(p - orig_center)`
- 矩阵构造为 `H = | R·s   offset - R·s·orig_center |`
  `         | 0 0 0          1           |`
- 通过`vtk.vtkMatrix4x4.SetElement()`逐元素写入，再`vtkTransform.SetMatrix()`设置
- 在`_on_obj_select_changed()`末尾添加`_apply_obj_transform_to_actor()`调用，确保选中物体时立即应用变换
- 保留`_obj_transforms`数据格式不变，不影响场景保存/加载

### [ID-7] 增加全局复位功能，将所有对象重置到初始位置，还有把原来的复位改成相机复位 ✅

- **状态**: 已验证修复
- **模块**: 视角控制 / 对象控制
- **严重程度**: 🔴 严重    
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29
**问题描述**:
增加全局复位功能，将所有对象重置到初始位置，还有把原来的复位改成相机复位

**修复内容**:
- 新增`_reset_all()`方法：遍历所有对象，重置offset/orig_center/yaw/pitch/roll/scale到初始值
- 将原来的"复位"改为"相机复位"(`_reset_camera()`)：仅重置相机位置到默认视角
- 菜单栏"视角"下同时提供"相机复位"和"全局复位(所有对象)"两个选项
- 工具栏中也添加了这两个按钮


### [ID-8] 增加删除图中的测距线和测角线的功能 ✅

- **状态**: 已验证修复
- **模块**: 测量工具
- **严重程度**: 🔴 严重    
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29
**问题描述**:
当有需要时，我想删除图中的测距线和测角线，但是目前没有这个功能

**修复内容**:
- 在`MeasurementTool`中添加`clear_all()`方法：清除所有测量图形
- 在`MeasurementTool`中添加`undo_last()`方法：撤销上一次测量
- 菜单栏"工具"和工具栏中均添加了"清除测量"和"撤销上一步测量"按钮

### [ID-9] 树的位置被terrain覆盖了，给他找个合适的位置 ✅

- **状态**: 已验证修复
- **模块**: 场景构建 (scene_builder.py)
- **严重程度**: 🟡 中等   
- **发现日期**: 2026-06-29
- **修复日期**: 2026-06-29
**问题描述**:
符合物理，不能悬在空中，不能被覆盖，要么在地上要么在山上

**修复内容**:
- 树的位置 (4.5, 2.0) 使用与地形相同的高斯函数公式计算该点的 terrain Z 值
- 树干底部 (local z=-0.5) 偏移 +0.5 使树根贴合地形表面
- 树冠在 terrain 之上自然生长，不会被覆盖

### [ID-10] 添加保存/载入数据功能 ✅

- **状态**: 已验证修复
- **模块**: 数据持久化 (main_window.py)
- **严重程度**: 🔴 严重  
- **发现日期**: 2026-06-30
- **修复日期**: 2026-06-30
**问题描述**:
添加功能可以保存数据，分为两个文件夹，保存的数据文档必须为json文件，
第一个数据保存所有aircraft的数据（目前为aircraft和aircraft2）
但是以后添加新的aircraft后使用这个功能也能同样储存，
另一个数据保存所有地形，
这两个数据分别存到两个不同的文件夹里，但是文件名需要展示他们之间时同一组数据
需要达到的效果是当我在pycista上加载数据时，要原封不动的加载出所有之前保存的状态

**修复内容（第一次）**:
- `_save_data()`: 用户输入基础名称(如"mission1")，自动创建配对JSON到`data/aircraft/`和`data/terrain/`
- `_load_data()`: 选择飞行器JSON，自动加载配对地形JSON
- 筛选`"aircraft" in name.lower()`确保自动适配未来新增的aircraft

**再次修改**：
目前aircraft保存好的数据可以加载，但是terrain保存好的数据加载后没有回到之前的状态，比如我先保存再移动再加载
terrain没有回到之前的位置
还有目前有四个保存加载功能，我只要两个，一个‘保存数据（json）’和一个‘加载数据（json）‘分别具有对应功能
载入飞行数据只有json文件，缺少3Dplot文件

**修复内容（第二次 — 三项修复）**:
1. **terrain加载还原修复**: 在`_load_data()`中修改地形网格点后，调用`self._rebuild_actor("terrain")`强制VTK mapper重新读取更新后的点坐标和scalars数组，解决网格数据不刷新问题
2. **菜单简化**: 移除了`保存场景(JSON)/加载场景(JSON)`和工具菜单中的`保存飞行数据/载入飞行数据`，只保留`保存数据/加载数据`两个菜单项。飞行数据保存/载入按钮保留在左侧面板中
3. **3Dplot文件**: `_save_flight_data()`保存JSON时，同时使用matplotlib生成3D散点图（红色航点编号 + 蓝色飞行路径），保存为`_3dplot.png`，不依赖额外库（matplotlib不可用时自动跳过）

**修复内容（第三次 — 2026-06-30 验证修复）**:
1. **`_load_data()`缺少`_refresh_flight_combo()`调用**: 载入数据后，飞行器选择下拉框未刷新。修复：在`_load_data()`末尾添加`self._refresh_flight_combo()`调用
2. **`_load_data()`未更新UI滑块**: 载入transform数据后，对象控制面板的滑块值仍显示旧值。修复：添加`self._on_obj_select_changed(self._obj_combo.currentIndex())`触发滑块刷新
3. **`_save_flight_data()`缺少段结构校验**: 保存飞行数据时，未验证segments每个段是否包含必要字段。修复：添加required_keys子集校验

### [ID-11] 路径点动态飞行 ✅

- **状态**: 已验证修复
- **模块**: 路径规划 / 飞行动画
- **严重程度**: 🔴 非常严重  
- **发现日期**: 2026-06-30
- **修复日期**: 2026-06-30
**问题描述**:
路径点添加新功能，让飞机从每个路径点逐个飞过，速度要合理，飞之前能选择飞机
添加完路径点后，飞机从每个路径点按直线穿过，起始点为飞机的原位置

格外注意当飞机往下飞时（或任意切换状态如转弯等），对应的飞机状态（不止xyz也包括yaw，pitch，raw）也相应改变，
如飞机往下飞后俯仰角会>0，具体是多少请计算得出

添加保存飞行路径数据功能，数据存在第三个文件夹里，包括一个json文件可以加载复现飞行动态，以及一个3Dplot描述飞行路线

**修复内容**:
- 实现`_start_flight()`: 从飞机当前位置到所有路径点构建路径段，每段计算yaw(atan2(dx,dy)%360)和pitch(atan2(-dz,horiz_dist) clamped ±90)，50ms QTimer驱动
- 实现`_flight_tick()`: 线性插值位置，更新transform偏移量+yaw/pitch/roll，同步滑块UI，调用_apply_obj_transform_to_actor刷新VTK
- 实现`_stop_flight()`: 停止timer，恢复UI控件状态
- 实现`_ensure_flight_dir()`: 在项目根目录创建data/flight/文件夹
- 实现`_save_flight_data()`: 保存JSON飞行数据，同时生成_3dplot.png（matplotlib可用时）
- 实现`_load_flight_data()`: 载入JSON并重放动画
- 实现`_refresh_flight_combo()`: 随_init_scene自动填充飞机选择下拉框

### [ID-12] 添加编程算法概要.md文件 ✅

- **状态**: 已验证修复
- **模块**: 文档
- **严重程度**: 🟡 中等  
- **发现日期**: 2026-06-30
- **修复日期**: 2026-06-30
**问题描述**:
把每个py文件所用的编程思路放到同一个.md文件里，方便以后查看。公司需要这个软件的编程思路和实现过程。越详细越好
比如在scene.py里，地形创建用了什么方法, 其他.py也按照这个思路写详细编程实现过程和算法。

**修复内容**:
- 创建 `编程算法概要.md` 文件，覆盖全部 7 个 Python 模块
- 详细记录地形生成算法（高斯函数叠加 + 河道雕刻）、飞机建模（多部件 merge 粘合）、坐标转换（Qt-VTK Y 轴翻转）、姿态控制（numpy 齐次变换矩阵）、飞行动画（分段线性插值 + 欧拉角）等核心算法
- 文档约 300 行，包含数学公式描述和代码示例


### [ID-13] 保存加载数据问题 ✅

- **状态**: 已验证修复
- **模块**: 数据持久化 (main_window.py)
- **严重程度**: 🟡 中等  
- **发现日期**: 2026-06-30
- **修复日期**: 2026-06-30
**问题描述**:
现在载入aircraft数据后，地形也会载入。我需要的是aircraft只保存aircraft数据，地形只保存地形数据。

**修复内容**:
- 将`_load_data()`拆分为`_load_aircraft_data()`和`_load_terrain_data()`两个独立方法
- 根据用户所选JSON文件的所在目录（`data/aircraft/`或`data/terrain/`）自动判断数据类型
- 载入aircraft JSON时仅恢复飞机transform
- **载入terrain JSON时恢复所有非aircraft对象**（terrain网格高程、river/vegetation/bird/tree/custom等的transform、config、camera）
- `_save_data()`中terrain JSON新增`"objects"`字段，保存全部非aircraft对象的transform


### [ID-14] 在路径点处瞬间切换姿态（不连续） ✅

- **状态**: 已验证修复
- **模块**: 飞行动画/路径规划
- **严重程度**: 🔴 非常严重
- **发现日期**: 2026-06-30
- **修复方式**:
  - 位置：使用 Catmull-Rom 样条插值替代线性插值，生成 C1 连续的光滑曲线通过所有路径点
  - 航向角：在相邻段间进行角度插值（处理 0/360 绕回），每段内从当前段方向平滑过渡到下一段方向
  - 俯仰角：在相邻段间线性插值，消除段切换时的俯仰突变
  - 边界处理：首尾段使用镜像控制点保持切线方向


### [ID-15] 穿过路径点的时间一致但速度不一致（因为路径距离不同） ✅

- **状态**: 已验证修复
- **模块**: 飞行动画/路径规划
- **严重程度**: 🟡 中等
- **发现日期**: 2026-06-30
- **修复方式**:
  建立了系统性的飞机速度计算数学模型，替代固定分段步数：
  - **恒定巡航速度**：平飞时 V₀ = 5.0 u/s，每段步数 = 距离 / V₀ / 帧间隔，不再受段长度影响
  - **俯仰因子 k_pitch**：max(0.30, 1.0 − 0.35⋅sin(pitch))，爬升减速、俯冲加速（见算法文档）
  - **转弯因子 k_turn**：max(0.30, 1.0 − 0.006⋅|yaw_rate|)，急转弯大幅减速
  - **两遍计算**：第一遍估算 yaw_rate，第二遍用完整速度模型确定每段步数

### [ID-16] 添加一种添加路径点的方式 ✅

- **状态**: 已验证修复
- **模块**: 路径规划/UI
- **严重程度**: 🟢 轻微
- **发现日期**: 2026-06-30
**问题描述**:
原来的"或输入坐标"改为"精准添加路径点"按钮，提供双步骤向导对话框精准添加路径点：

**修复内容**:
- 将 `QLabel("— 或输入坐标 —")` 替换为 `QPushButton("精准添加路径点")`
- 新增 `WaypointPreciseDialog(QDialog)` 双步骤向导类：
  - **Step 1 — XY 选择**: 嵌入式 matplotlib 2D 散点图（-10~10 网格），点击 canvas 生成坐标，或直接输入 X/Y spinbox，双向同步
  - **Step 2 — Z 选择**: 水平条状图 + Z spinbox，点击或输入高度
- 点击"确定"后调用 `_add_waypoint()` 添加路径点并刷新 3D 场景
- 原有坐标输入 spinbox 和"添加坐标路径点"按钮保持不变
- matplotlib 不可用时弹出警告提示安装

### [ID-17] 精准添加修改 ✅

- **状态**: 已验证修复
- **模块**: 路径规划/UI
- **严重程度**: 🔴 严重
- **发现日期**: 2026-06-30

**修复内容**:
重构 `WaypointPreciseDialog`，从单点输入升级为多点路径输入：

**Step 1 — 多点 XY 选择**:
- 点击 matplotlib 2D 散点图依次添加多个点，每个点按 A/B/C/D... 顺序编号
- 点之间用连线连接，预览路径走向
- X/Y QDoubleSpinBox 编辑当前选中（最后添加）点的坐标，实时更新图表
- "撤销上一点"按钮删除最后添加的点
- 至少 2 个点才能进入下一步

**Step 2 — 距离-高度剖面图**:
- X 轴为累积路径距离（AB 段距离、BC 段距离...），Y 轴为 Z 高度
- 折线图连接各点 Z 值，点击图表切换选中点并设置其 Z 高度
- 当前选中点信息显示为 "当前点: [标签], 距离: [累积距离]"
- 所有点默认 Z = 0，可逐点调整

**结果**:
- 点击"确定"一次性将所有点（含 XY 坐标和 Z 高度）添加为路径点
- 调用 `_show_path()` 刷新 3D 样条曲线

### [ID-18] 添加图层 ✅

- **状态**: 已验证修复
- **模块**: 图层管理
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03

**问题描述**:用图层（e.g 草地，沙地，土地）对地形进行管理和渲染，实现对不同地形的可视化显示。移除原有的高度-based colormap，改为物理颜色图层。

**修复内容**:
- 移除原有的基于高度的 "terrain" colormap（terrain 条目改为隐藏数据网格，不再参与渲染）
- 在 `scene_builder.py` 中新增 `build_terrain_layer_meshes()` 函数，通过 elevation 阈值划分 3 个地形图层：
  - • **沙地 (layer_sand)**: 低海拔区域 (bottom 25%)，黄色渐变 (浅黄→金黄→橙黄)
  - • **草地 (layer_grass)**: 中海拔区域 (25%-65%)，绿色渐变 (浅绿→森林绿→深绿)
  - • **土地 (layer_earth)**: 高海拔区域 (top 35%)，棕色渐变 (土黄→棕→深棕)
- 每个图层使用 `threshold()` 从地形表面提取，保留 elevation 标量，使用 3 色渐变 cmap 实现渐变着色
- 在 `main_window.py` 中将左侧 "场景元素" 码头替换为 "图层管理" 码头：
  - 每个图层（沙地/草地/土地/河流）有复选框 (可见/隐藏) + 不透明度滑块 (0%-100%)
  - 滑块回调将不透明度持久化到 `scene_objects.params`，切换图层开关时保持用户设置
  - 下方保留场景树用于对象选择
- 右侧 "图层控制" 码头排除地形图层（由左侧专属管理），避免重复控制
- `_load_terrain_data()` 加载地形数据后自动调用 `_rebuild_terrain_layers()` 重建图层网格

### [ID-19] 精准添加再次修改 ✅

- **状态**: 已验证修复
- **模块**: 路径规划/UI
- **严重程度**: 🔴 严重
- **发现日期**: 2026-06-30
- **修复日期**: 2026-07-03
**问题描述**:
我需要坐标可以在下方同时存在并可以进行删除管理，比如我添加了 A/B/C/D... 等多个点，我可以在下方同时查看这些点的坐标，也可以删除其中任何一个点。还有比如说我已经到step2，然后对ABC进行了更改，然后回到step1重新检查后，再回到step2之前的进度还在，不需要重新添加，要有memory。还有step2里面选择z时最好可以滑动选择而不是点击选择，这样更加精准。

**修复内容**:
- **坐标列表管理**: 在 Step1 散点图下方新增 `_point_table` (QTableWidget)，列：点/A/B/C…/X/Y/操作。每行显示点编号 + X/Y 坐标 + "删除"按钮，`_delete_point(idx)` 删除指定点后自动重编号并更新图表
- **Step 间 Memory**: `_go_step2()` 改为条件初始化 `_z_values`（仅在数组长度不匹配时重置），`_undo_last_point()` 和 `_delete_point()` 同步删除 `_z_values` 对应项。来回切换 Step1↔Step2 保留所有 Z 值和选中状态
- **Z 高度滑块**: 在 Step2 距离-高度剖面图中移除 click-to-set-Z 行为，新增水平 QSlider (0–1000 步，映射 -20…20)。滑动时实时更新选中点的 Z 值并重绘剖面图。滑块与 QDoubleSpinBox 双向同步

### [ID-20] 添加时间轴，添加飞机属性 ✅

- **状态**: 已验证修复
- **模块**: 路径规划/飞行动画/UI
- **严重程度**: 🔴 严重    
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03
**问题描述**:
在选择开始飞行后，可以选择或不选择跳出来一个独立的窗口，更加沉浸式的看飞行动画，我需要在原来和新加的窗口都添加一个时间轴，用于展示不同时间点的飞机位置和姿态。时间轴可以拖动，并且可以显示不同时间点的飞机位置和姿态。实现这个你可能得事先计算出总时间，同时，我需要添加一个按钮，用于保存当前时间点的飞机位置和姿态。还有给飞机本身添加一个属性-当前路径的所有路径点的坐标，需要实现的是当我开始飞行后，飞机的起点不是原来的位置而是第一个路径点，并且飞机的终点是最后一个路径点。并且这个路径点是可以给另一个飞机当为属性，也就是说可以给另一个飞机使用同一个路径点。

**修复内容**:
- **Timeline**: 添加可拖动的QSlider时间轴，飞行开始时显示在路径规划面板中。预计算总飞行时间(ms)，滑块范围映射到整个飞行时长。滑块上方显示路径点关键帧标记(⬤1 ⬤2 …)。支持拖动寻址——拖动时暂停定时器、更新飞机位置/姿态、释放后恢复。
- **保存当前姿态**: 添加"保存当前姿态"按钮，记录当前飞机offset/yaw/pitch/roll和时间戳到`_saved_flight_states`列表。
- **独立飞行窗口**: 点击"开始飞行"时弹出QMessageBox询问"独立窗口观看飞行?"。选择"是"则创建`FlightWindow`(QDialog)，包含独立的ClickablePlotter渲染器(深色背景`#1a1a2e`+地形上下文+飞机actor+灯光)。飞行中自动跟随飞机摄像机。
- **飞机路径属性**: 新增`_aircraft_waypoints`字典(名称→路径点列表)作为每架飞机的共享路径属性。`_get_aircraft_waypoints()`先查飞机自有路径，回退到全局路径点。`_start_flight()`直接使用飞机路径点构建飞行路径。
- **飞机起点/终点**: 飞行开始时飞机瞬移到路径点[0]作为起点，飞行结束停在路径点[-1]（原行为是从飞机当前位置飞行到所有路径点）。
- **复制路径到...**: 添加"复制路径到..."按钮，弹出飞机选择列表，将当前全局路径点复制到目标飞机的`_aircraft_waypoints`中。
- **代码重构**: 将`_flight_tick()`中的位置/姿态计算提取为`_compute_flight_state_at()`和`_apply_flight_state()`，使时间轴寻址和定时器tick共享同一插值逻辑。`FlightWindow.update_position()`复制与主窗口相同的变换矩阵到独立渲染器的actor。

**2026-07-03 修复：时间轴点击跳转（视频播放器式）**:
- `QSlider.sliderMoved` → `valueChanged`，支持点击轨道跳转（非拖拽）
- 点击任意位置 → 自动跳转到该时刻并继续播放
- 拖拽时暂停，释放后恢复，行为完全对齐视频播放器

**2026-07-03 修复：FlightWindow 独立窗口崩溃**:
- Root cause：二级 QVTKRenderWindowInteractor 创建后立即调 render()，在部分 OpenGL 驱动下因上下文未就绪崩
- StructuredGrid 在二级渲染器中存在兼容性问题
- Fix: terrain 提取为 PolyData 再添加；剥离 scalars/cmap/clim（terrain 已改为纯色）；render() 延迟 50ms 到 `_init_view()`；包裹全部渲染操作为 try/except 容错




### [ID-21] 地形颜色 / 植被图层 / ID-3拾取升级 / FlightWindow崩溃 ✅

### [ID-22] 删除东北天坐标输入 + UI清理 + 隐藏高程标量条 + 时间轴常驻 + FlightWindow二次修复 ✅

- **状态**: 已验证修复
- **模块**: 路径规划 / 图层管理 / 飞行动画 / UI
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03
**问题描述**:
1. 删除"东北天"直接坐标输入spinbox和"添加坐标路径点"按钮
2. 图层管理初始值设为100%透明(opacity=0.0)
3. 地形图层的PyVista自动elevation标量条无法删除(内置),改用show_scalar_bar=False隐藏
4. 时间滚动条应在主界面始终可见(不再飞行时才显示)
5. FlightWindow独立窗口仍然崩溃(第一次修复不彻底)
6. 删除上一轮残留的_show_path/_save_current_state代码

**修复内容**:
1. **删除坐标输入**: 移除`_wp_x/_wp_y/_wp_z`三连spinbox及`btn_add_coord`按钮，删除`_add_wp_from_coords()`方法，清理`_set_coord_system()`中的spinbox引用
2. **图层初始透明**: 图层管理滑块初始值`1.0→0.0`，`scene_builder.py`中图层params`opacity: 1.0→0.0`
3. **隐藏标量条**: `scene_builder.py`所有terrain layer params添加`show_scalar_bar: False`，抑制PyVista自动添加的elevation标量条(color bar随之消失)
4. **时间轴常驻**: 移除`_timeline_container.hide()`和_start_flight/_stop_flight中的show()/hide()，时间轴始终显示在主界面
5. **FlightWindow二次修复**: 创建`FlightPlotter(ClickablePlotter)`子类，延迟所有VTK渲染至showEvent，render在`_flight_ready`标志为True前为no-op，设置`render_window.SetOffScreenRendering(1)`避免macOS双QVTK窗口OpenGL上下文冲突
6. **代码清理**: 移除_show_path残留stub，移除所有_populate_tree/_refresh_layers调用，移除图层控制dock

### [ID-23] FlightWindow 独立窗口崩溃 (macOS双QVTK) ✅

- **状态**: 已验证修复
- **模块**: 飞行动画 / FlightWindow
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03
**问题描述**:
macOS上开启第二个QVTKRenderWindowInteractor时OpenGL上下文冲突，二级窗口闪1秒后崩溃。Root cause: VTK在macOS上不支持两个活动OpenGL上下文同时存在。

**修复方式**: 彻底删除独立窗口方案。移除 `FlightWindow`、`FlightInfoPanel`、相机跟随模式、`_open_flight_window`/`_on_flight_window_closed`/`_restore_camera` 方法、「独立窗口观看飞行?」询问对话框。飞行始终在主视口进行。


### [ID-24] 添加FlightPlotter时误吞ClickablePlotter鼠标事件方法 ✅

- **状态**: 已验证修复
- **模块**: 路径规划/3D交互
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03
**问题描述**:
编辑添加FlightPlotter类时代码替换出错，ClickablePlotter的`_to_vtk_display`/`mousePressEvent`/`mouseReleaseEvent`/`mouseMoveEvent`/`_process_click`五个方法被意外移入FlightPlotter。由于MainWindow使用ClickablePlotter，鼠标在3D视口中完全不响应点击，导致无法添加路径点、无法选择物体。

**修复内容**:
- 将5个鼠标事件方法恢复到ClickablePlotter类
- FlightPlotter仅保留`__init__`/`render`/`showEvent`/`_first_render`四个覆写，鼠标事件从父类继承


### [ID-25] FlightWindow 独立窗口 + 相机跟随完全删除 ✅

- **状态**: 已验证修复
- **模块**: 飞行动画 / UI
- **严重程度**: 🟡 中等
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03
**问题描述**:
macOS双QVTK问题无法通过OffScreenRendering等方案彻底解决，FlightPlotter方案仍存在崩溃风险。用户要求删除整个独立窗口逻辑。

**修复内容**:
- 删除「独立窗口观看飞行?」QMessageBox对话框
- 删除FlightInfoPanel类（非QVTK替代方案）
- 删除_open_flight_window() / _on_flight_window_closed() / _restore_camera() 方法
- 删除_camera_follow_active / _saved_camera 状态变量
- 清理_apply_flight_state(): 移除面板更新和相机跟随代码
- 清理_stop_flight(): 移除面板关闭和相机恢复代码
- 飞行始终在主视口进行

### [ID-26] 沙地/草地/土地图层默认不勾选 ✅

- **状态**: 已验证修复
- **模块**: 场景构建 / 图层管理
- **严重程度**: 🟢 轻微
- **发现日期**: 2026-07-03
- **修复日期**: 2026-07-03
**问题描述**:
沙地(layer_sand)、草地(layer_grass)、土地(layer_earth)三个地形图层默认勾选（visible=True），用户希望默认不勾选但保持完全不透明。

**修复内容**:
- scene_builder.py: 三个图层的 `visible` 从 `True` 改为 `False`
- scene_builder.py: 三个图层的 `opacity` 从 `0.0` 改为 `1.0`（勾选后直接显示全彩）
- main_window.py _init_scene(): 添加 `if obj.get("visible", True)` 过滤，不勾选的对象启动时不添加到3D视口


### [ID-27] 增加编队功能 ✅

- **状态**: 已验证修复
- **模块**: 飞行动画/UI
- **严重程度**: 🟡 中等  
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06
**问题描述**:
增加编队功能，用户可以在主界面选择多个aircraft，将它们编队到一起飞行。

**修复内容**:
- 在`_setup_docks()`的保存/载入行添加`编队飞行`可切换按钮(QPushButton, checkable)，选中时高亮蓝色
- 按钮下方显示可勾选的QListWidget，列出所有aircraft，勾选即加入编队
- 第一个勾选的为**领队(leader)**，其余为**僚机(follower)**
- `_start_flight()`: 编队模式下使用领队的路径点，计算每架僚机相对于领队的初始偏移量，将领队和所有僚机瞬移到起始位置
- `_apply_flight_state()`: 每帧更新领队位置+姿态后，计算每架僚机位置 = 领队位置 + 初始偏移量，并应用变换
- 飞行期间编队按钮禁用，停止飞行后恢复
- 状态栏显示"编队 N机"指示编队规模


### [ID-28] PyVista `extract_surface(algorithm=None)` 崩溃 ✅

- **状态**: 已验证修复
- **模块**: DEM导入 / 场景构建
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题描述**:
运行 `3DSceneSoftware_test2.py` 时崩溃，错误为 `TypeError: extract_surface() got an unexpected keyword argument 'algorithm'`。
同时 DEM 导入功能（导入模型 → DEM 模型）也因相同原因崩溃。

**根因分析**:
`scene_builder.py` 中 `build_terrain_layer_meshes()` 和 `dem_loader.py` 中 `build_dem_scene()` 均调用了
`grid.extract_surface(algorithm=None)`。PyVista 的 `extract_surface()` 在旧版本（如 Python 3.9 环境下的版本）
不接受 `algorithm` 关键字参数。虽然 0.48.x 版本支持该参数，但未来默认值会从 `'dataset_surface'` 改为 `None`，
旧的 API 调用方式在新旧版本间不兼容。

**修复内容**:
- `scene_builder.py` line 305: `grid.extract_surface(algorithm=None)` → `grid.extract_surface()`
- `dem_loader.py` line 229: 同上移除 `algorithm=None`

**验证结果**:
- ✅ 43/43 单元测试全部通过
- ✅ DEM 加载正常：931×646 网格，Z 范围 -36~1128m
- ✅ DEM 场景构建正常：terrain + 3 图层 + 2 架 aircraft
- ✅ 飞机缩放 500×后尺寸：1125×1350×255，Z≈1048m
- ✅ 原有默认场景功能不受影响（测试全部通过）


### [ID-29] 多项功能新增与改进（鼠标坐标精度/菜单图层/Z夸张/7000m高度/全地形覆盖/XY自动范围）

- **状态**: 已验证修复
- **模块**: 全局（场景构建 / UI / 路径规划 / DEM导入）
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题/需求描述**:

1. **鼠标点击偏移**：3D视口中点击添加路径点时，路径点出现位置与鼠标点击位置不符。之前ID-3曾修复Y轴翻转问题，但`_to_vtk_display()`公式与QVTK内部`_setEventInformation`公式在Retina屏幕上仍有1像素偏差。
2. **图层管理移至菜单**：需要将图层管理（沙地/草地/土地/河流/植被的复选框+透明度滑块）从右侧Dock面板移除，放到左上角菜单栏中，功能完全不变。
3. **默认地形全覆盖**：默认地形（非DEM）的沙地/草地/土地图层仅覆盖底部70%高程范围，顶部30%无覆盖层。需要像DEM一样覆盖100%高程范围。
4. **精准路径点XY自动范围**：`WaypointPreciseDialog`的XY散点图范围（±10）和Z剖面图范围（±20）为硬编码，在DEM大范围场景中不适用。需要根据terrain mesh的bounding box自动检测范围。
5. **DEM导入Z夸张选项**：DEM导入时垂直夸张系数（vert_exag）为硬编码2.0，用户需要能在导入时选择该值。
6. **飞机默认高度7000m**：DEM场景中飞机默认置于Z=1000m，对于大范围DEM地形（如ASTER GDEM 136km对角线）过低，需改为7000m。

**修复内容**:

1. **鼠标坐标精度修复** (`src/main_window.py`):
   - `_to_vtk_display()`: 将Y坐标转换公式从`win_height - round(y*scale) - 1`改为`round((self.height() - y - 1) * scale)`，与QVTK的`_setEventInformation`完全一致，消除Retina屏幕上的1像素偏差。

2. **图层管理移至菜单** (`src/main_window.py`):
   - 新增`_build_layer_menu_action()`方法：为每个图层构建QWidgetAction（内嵌QCheckBox+透明度QSlider），保持与Dock面板完全相同的功能。
   - 在`_setup_menus()`中添加"图层 (&L)"菜单，Alt+1~5快捷键。
   - `_setup_docks()`中移除图层管理Dock面板（QDockWidget("图层管理")完全删除）。
   - 所有回调方法（`_toggle_terrain_layer`、`_on_terrain_opacity`、`_refresh_terrain_ui`）保持不变。

3. **默认地形全覆盖** (`src/scene_builder.py`):
   - `build_terrain_layer_meshes()`: 将earth图层阈值上界从`z_min + elev_range * 0.70`改为`z_max + 1.0`，使沙地(0-20%)+草地(20-45%)+土地(45-100%)覆盖全部高程范围，与DEM行为一致。
   - 删除不再使用的`earth_max`变量。

4. **精准路径点XY自动范围** (`src/main_window.py`):
   - `WaypointPreciseDialog.__init__()` 新增`terrain_extent`参数`(xy_half, z_half)`。
   - 用实例变量`_xy_limit`、`_z_limit`、`_spin_xy_range`替代所有硬编码范围（±10、±20）。
   - 所有Z相关方法（`_redraw_z`、`_on_z_click`、`_on_z_slider_changed`、`_on_z_spin_changed`）均使用动态范围。
   - `MainWindow._compute_terrain_extent()`: 从terrain mesh的bounding box自动计算范围，带1.2×XY/1.5×Z边距。
   - `_open_precise_wp_dialog()`: 调用`_compute_terrain_extent()`并传递给dialog。

5. **DEM导入Z夸张选项** (`src/main_window.py`):
   - `_import_dem_model()`: 加载DEM数据后，使用`QInputDialog.getDouble()`让用户选择垂直夸张系数（0.1~20.0，默认2.0）。
   - 将用户选择的值传递给`build_dem_scene(vert_exag=...)`和确认对话框显示。
   - 确认对话框现在显示"垂直夸张: X.X×"信息。

6. **飞机默认高度7000m** (`src/dem_loader.py`, `src/main_window.py`):
   - `build_dem_scene()`: 默认参数`aircraft_z`从`1000.0`改为`7000.0`。
   - `_import_dem_model()`: 调用`build_dem_scene(aircraft_z=7000.0)`。
   - 确认对话框提示文本从"Z=1000m"改为"Z=7000m"。
   - DEM导入后的Z滑块范围从`-500~3000`改为`-500~10000`。
   - `dem_loader.py` docstring更新。

**验证结果**:
- ✅ 43/43 单元测试全部通过
- ✅ `_to_vtk_display`公式与QVTK完全一致
- ✅ 图层管理在菜单中功能正常（复选框+透明度滑块）
- ✅ 默认地形地球图层覆盖100%高程范围
- ✅ WaypointPreciseDialog根据terrain mesh自动调整XY/Z范围
- ✅ DEM导入时可选Z夸张系数
- ✅ DEM场景飞机初始高度改为7000m

---

### [ID-30] DEM飞机放大（500×→2000×）

- **状态**: 已验证修复
- **模块**: DEM导入
- **严重程度**: 🟡 一般
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题**: DEM场景中飞机默认缩放500×，在ASTER GDEM（~136km对角线）尺度下仍难以辨认。

**修复**: `src/dem_loader.py` 中 `AIRCRAFT_DEFAULT_SCALE` 从 500 改为 2000，飞机长度约4.8km，在1296px视口下约44px，清晰可见。

---

### [ID-31] DEM对象控制优化（仅显示aircraft1/2/terrain，默认aircraft1）

- **状态**: 已验证修复
- **模块**: 对象控制/UI
- **严重程度**: 🟡 一般
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题**: DEM场景中对象控制下拉框显示所有场景对象（包括不必要的river/vegetation/bird/tree），且顺序无优先级。

**修复**: 
- `_refresh_obj_combo()`: DEM模式下仅显示aircraft、aircraft2、terrain，按此顺序排列
- `_refresh_scene_objects_ui()`: DEM模式场景对象复选框同理
- 默认选中aircraft1
- 新增 `_is_dem_scene()` 方法检测DEM场景（检查terrain extra中是否存在X/Y网格数据）

---

### [ID-32] DEM相机视图修复（俯视/侧视/复位使用地形动态范围）

- **状态**: 已验证修复
- **模块**: 视角
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题**: `_set_view()` 硬编码 `dist=25`，在DEM大场景（~100km）中俯视/侧视完全不可用。`_reset_camera()` 同样使用小场景硬编码位置。

**修复**: 
- `_set_view()`: 使用 `_compute_terrain_extent()` 动态计算相机距离 `max(xy_half * 2.5, 25)`
- `_reset_camera()`: 同样使用地形动态范围设置相机位置
- `_reset_all()`: 继承 `_reset_camera` 动态行为

---

### [ID-33] DEM保存/载入修复（保存X,Y网格实现完整重构）

- **状态**: 已验证修复
- **模块**: 数据持久化
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题**: DEM场景保存时仅保存 `original_z` 数组，不保存X/Y坐标网格。载入时无法完整恢复DEM地形。

**修复**:
- `_save_data()`: DEM场景额外保存 `X` 和 `Y` 二维数组到JSON
- `_load_terrain_data()`: 检测保存数据中是否有X/Y，有则重建 `StructuredGrid`，无则保持原有逻辑

---

### [ID-34] 删除STL/OBJ导入，仅保留DEM导入

- **状态**: 已验证修复
- **模块**: 导入
- **严重程度**: 🟡 一般
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题**: 「文件」菜单同时有「导入模型 (STL/OBJ)」和「导入 DEM 模型」两个选项，STL/OBJ导入功能在此项目中不再需要。

**修复**: 从 `_setup_menus()` 中移除 `"导入模型 (STL/OBJ)..."` 菜单项，保留 `_import_model()` 方法（但不再通过菜单调用）。

---

### [ID-35] 3D交互鲁棒性改进（picker精度提升/None坐标处理）

- **状态**: 已验证修复
- **模块**: 3D交互
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**问题**: `_on_3d_click` 中使用 `np.linalg.norm(world_pos) < 1e-6` 检查无效点击，但在DEM大场景中picker有时无法精确命中导致world_pos为(0,0,0)，路径点和测量交互被静默忽略。

**修复**:
- `_process_click()`: VTK CellPicker tolerance 从 0.001 提升到 0.005
- 删除了最后保底的 `vtkWorldPointPicker`（返回焦平面深度，DEM中完全错误）
- 改用 `None` 表示picker失败
- `_on_3d_click()`: 改用 `world_pos is None` 检查替代向量范数检查

---

### [ID-36] 新增图层管理对话框（XY绘图工具+矩形/圆形/多边形选区）

- **状态**: 已验证修复
- **模块**: 图层管理/UI
- **严重程度**: ⭐ 新功能
- **发现日期**: 2026-07-06
- **修复日期**: 2026-07-06

**需求**: 需要一个统一的图层管理界面，支持在XY平面图上绘制选区（矩形/正方形/三角形/圆形/自定义多边形），将地形图层（沙地/草地/土地）仅添加到选区范围内。同时支持普通场景中的河流/植被快捷显隐。

**实现**:
- 新增 `src/layer_dialog.py`: `LayerManagementDialog` (QDialog)
- 两页式QStackedWidget：
  1. 第一页：图层选择（DEM: 沙地/草地/土地，普通: +河流/植被）
  2. 第二页：XY平面图 + 绘图工具栏
- 绘图工具：矩形、正方形(中心+半径)、三角形(3顶点)、圆形(中心+半径)、自定义多边形(多点击+闭合)
- 左侧面板：形状列表（选中可调透明度、可删除）
- 确认后调用 `_apply_layer_shapes()` 提取选区内的mesh子区域
- 使用 matplotlib Path.contains_points 进行点-多边形包含测试

---

### [ID-37] UI重构：场景树+属性面板+坐标简化+ASC导入导出+飞行/编队修复 ✅

- **状态**: 已验证修复
- **模块**: 全局
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-07
- **修复日期**: 2026-07-07

**修复内容**:
- 删除坐标系切换(ENU/FLU/NED/NWU)，坐标统一用 X/Y/Z 纯数值显示
- UI从splitter+scrollArea重构为：左dock(场景树) + 右dock(属性面板QStackedWidget) + 底dock(时间轴+坐标信息)
- 属性面板按树选中项动态切换(飞机姿态/路径点/飞行动画)，不再包QScrollArea
- 删除场景设置根节点(只剩飞行平台/路径规划/动画与任务三个根节点)
- 删除"地图背景切换"按钮
- "添加新飞机平台"→"添加新的飞机"，删除"+ 添加"里的"导入DEM模型数据"
- 树中路径规划新增"精准添加路径"和"清除所有路径点"
- 文件菜单新增"导入ASC格网数据..."和"导出ASC格网数据..."
- ASC解析支持ESRI ASCII Grid格式(ncols/nrows/xllcorner/yllcorner/cellsize/NODATA_value)
- 修复飞行按钮无响应(_btn_start_flight未接.clicked)
- 修复编队僚机重叠(所有僚机之前共用同一个FORMATION_TRAIL_DIST)
- 修复编队DEM场景距离(×AIRCRAFT_DEFAULT_SCALE×0.005)
- 修复飞机顺位命名(最小空号)
- 修复_add_new_aircraft崩溃(import不存在)
- 修复DEM场景新建飞机不可见(scale/position适配)
- 修复DEM场景对象控制combo不显示aircraft3+(硬编码列表)
- 飞机命名规则：号1="aircraft", 号2+="aircraft2"...


### [ID-38] 新增操作日志 + 场景设置属性（只读/自动更新）+ 路径点模式恢复

- **状态**: 已验证修复
- **模块**: UI / 全局
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-08
- **修复日期**: 2026-07-08

**变更内容**:

1. **操作日志 (底部左下)**:
   - 底部 Dock 重构为左右分区：左（stretch=2）操作日志 + 右（stretch=1, maxWidth=400）坐标信息
   - 新增 `_log_action(msg)`：时间戳 + 自动滚动到底部
   - 已插桩 20+ 方法：增删飞机/路径点/图层、切换视角、加载地形、复位、选中对象、导入 ASC/DEM、测距测角、编队、保存/载入飞行数据、碰撞检测等
   - 对象位置变换滑块的日志带 1.5s 去抖（`_pending_transform_log` + 单次 QTimer）

2. **场景设置 → 场景信息独立窗口**:
   - 场景树"场景设置"节点下新增"场景信息（双击）"子节点
   - 双击弹出独立 `SceneSettingsDialog` 窗口，1s 定时自动刷新
   - 显示：作者、日期、坐标系、地形尺寸、Z偏移、边界、垂直夸张、相机视角
   - 坐标系从 hardcoded "WGS84" 改为动态读取 `config["coordinate_system"]` + DEM CRS
   - 垂直夸张从 DEM 导入时保存到 `config["elevation_scale"]`，对话框实时显示
   - 首屏刷新用 `QTimer.singleShot(0)` 延迟，避免 VTK/OpenGL 构造期死锁

3. **路径点模式恢复**:
   - `InteractionMode.WAYPOINT` 重新加入
   - "添加3D路径点（单击场景）" → `_toggle_wp_mode` → 单击 scene 放置路径点
   - "精准添加路径（双击）"和"清除所有路径点（双击）"保留

## 当前已知问题

<!-- 在此处按时间倒序添加问题条目 -->

| ID | 标题 | 模块 | 严重程度 | 状态 |
|----|------|------|----------|------|

---

### [ID-39] macOS Apple Silicon (Rosetta 2) 环境下 VTK 启动死锁 — 根因分析与修复 ✅

- **状态**: 已验证修复
- **模块**: 启动 / VTK 引擎
- **严重程度**: 🔴 严重（程序无法启动）
- **发现日期**: 2026-07-08
- **修复日期**: 2026-07-08

**问题现象**:
macOS Apple Silicon (arm64) 上安装 x86_64 版 Anaconda → Conda Python 和所有 pip 包均为 x86_64 二进制 → 运行在 Rosetta 2 转译层上。
执行 `python 3DSceneSoftware_test2.py` 时，`import vtk` 阶段直接卡死（30s+ 无响应），或需要 1-3 分钟才能完成，用户体验不可接受。

**根因分析（三层）：**

| 层 | 问题 | 数据 |
|----|------|------|
| 1. 环境 | Conda Python 是 x86_64 二进制，Rosetta 2 转译加载 | `file python` → `Mach-O 64-bit executable x86_64` |
| 2. 导入 | `import vtk` 触发 `vtk.py` wrapper，一次性导入 **144 个** VTK 子模块，每个加载一个 x86_64 `.so` 文件 | `vtk.py` 中有 144 行 `from vtkmodules.vtk... import *` |
| 3. 转译 | Rosetta 2 对每个 `.so` 的 Mach-O loader + 动态链接耗时 0.5–3 秒不等，部分模块（如 `vtkRenderingVolumeOpenGL2`）依赖链长，转译中在 `os.stat()` 处长期阻塞 | 实测每子模块 0.5–2s，144 个总需 1–5 分钟 |

**实际 VTK 使用量（7 个类，4 个子模块）：**

| 类 | 来源模块 | 使用次数 |
|----|----------|---------|
| `vtkCellPicker` | `vtkmodules.vtkRenderingCore` | 1× |
| `vtkPropPicker` | `vtkmodules.vtkRenderingCore` | 2× |
| `vtkWorldPointPicker` | `vtkmodules.vtkRenderingCore` | 2× |
| `vtkStringArray` | `vtkmodules.vtkCommonCore` | 1× |
| `vtkMatrix4x4` | `vtkmodules.vtkCommonMath` | 1× |
| `vtkTransform` | `vtkmodules.vtkCommonTransforms` | 1× |
| `vtkActor` | `vtkmodules.vtkRenderingCore` | 2× (layer_dialog) |

**修复方案**:
用 4 个精确的 `from vtkmodules.xxx import` 替代 `import vtk`：

```python
# 修复前: 加载 144 个模块，Rosetta 2 下 >60s
import vtk

# 修复后: 只加载实际使用的 4 个模块，0.3s
from vtkmodules.vtkCommonCore import vtkStringArray
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkRenderingCore import vtkCellPicker, vtkPropPicker, vtkWorldPointPicker
from vtkmodules.vtkRenderingCore import vtkActor  # layer_dialog.py
```

**修复效果**:

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| VTK 导入时间 | 30s+ 超时 / 1-5 min | **0.3s** |
| 模块加载数 | 144 个 | **4 个** |
| MainWindow 模块导入 | 26.5s (首次) | **1.7s** |
| MainWindow() 实例化 | - | **6.7s** |
| 总启动时间 | 不可用 | **~8s** |

**修改文件**:
- `src/main_window.py`: `import vtk` → 4 个精确导入 + 全部 `vtk.vtkXxx` → `vtkXxx`
- `src/layer_dialog.py`: `import vtk` → `from vtkmodules.vtkRenderingCore import vtkActor`

**附：Rosetta 2 为什么不稳定**:
Rosetta 2 的 AOT (Ahead-of-Time) 转译在首次加载动态库时触发，转译结果缓存在 `/var/db/oah/`。但 VTK 9.6.2 的部分 `.so` 文件（约 300 个）转译队列过大，macOS 的 `trustd`/`syspolicyd` 门禁检查（Gatekeeper）会对每个新 `.dylib` 做公证验证，与 Rosetta 2 的 `oahd` 转译守护进程竞争，导致 `os.stat()` 在 `pathlib.Path.glob()` 中阻塞（见堆栈：`vtkmodules/__init__.py → find_lib_path → Path.glob → os.stat` — 系统调用不返回）。

**根本解决**（推荐，非本次 fix）:
安装 native arm64 版 Miniforge/Conda，避免 Rosetta 2 转译开销。所有包的 `.so` 直接为 arm64 二进制，启动时间可从 8s 进一步降至 <2s。

---

### [ID-40] Enterprise重构: 事件驱动架构 + 魔法数字集中化 + UX防御 + DEM场景切换修复 ✅

- **状态**: 已验证修复
- **模块**: 全局架构
- **严重程度**: 🔴 严重
- **发现日期**: 2026-07-08
- **修复日期**: 2026-07-08

**问题清单**:

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | DEM加载后切换回默认场景，相机距离仍是DEM尺度（百万级），地形像小点 | `_init_scene()` 未重置 `camera_position`/`focal_point`/`clipping_range` | 硬重置到 `DEFAULT_CAMERA_POSITION` + `clipping_range=(0.1,100)` |
| 2 | DEM加载后无法重新调整Z夸张系数 | 无重调入口 | 新增 `_reapply_elevation_scale()` + 菜单项"重新调整Z垂直夸张" |
| 3 | 地形变更后跨模块状态不同步（路径点、滑块范围、combo不变） | 无事件广播机制 | 新增 `_on_terrain_changed(path)` — 自动清空路径点 + 刷新UI + 日志 |
| 4 | 点击"清除路径"/"删除飞机"时无选中项 → 静默无反应/易误解 | 无Toast反馈 | 添加状态栏提示"⚠ 请先在场景树中选中要删除的节点" |
| 5 | `_clear_waypoints()` 无操作时仍添日志 | 无条件执行 | 添加早期返回守卫 |
| 6 | 硬编码魔法数字散落各处（step=2, 飞机7000m, camera坐标等） | 无集中管理 | 统一提取到 `src/config.py` — 30+ 常量 |
| 7 | `_log_action()` 在 docks 初始化前被调用 → AttributeError | 时序依赖 | 添加 `hasattr` 守卫 |

**修复效果**:

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| DEM→默认场景切换后的相机距离 | 百万级（不可用） | 18 单位（即时可用） |
| Z夸张调整 | 需重新导入DEM | 菜单/场景树随时调整，即时生效 |
| 地形变更后路径点 | 残留旧路径 | 自动清空 + 日志记录 |
| 破坏操作无选中 | 静默 | 状态栏Toast提示 |
| 魔法数字管理 | 散落在各文件 | 统一在 config.py 管理 |

**修改文件**:
- `src/config.py` — 全面重写：30+ 魔法数字常量 + `dem_source_path` 配置项
- `src/main_window.py` — 新增 `_on_terrain_changed()`, `_reapply_elevation_scale()`, 修复 `_init_scene()` 相机重置, 修复 `_clear_waypoints()` 守卫, 修复 `_on_tree_delete()` Toast, 修复 `_log_action()` hasattr, 添加工具菜单"重新调整Z垂直夸张"
- 导入更新: 使用 `DEFAULT_CAMERA_POSITION`, `CELL_PICKER_TOLERANCE`, `SAVE_DIR_*` 等集中化常量

---

## 已关闭问题

| ID | 标题 | 模块 | 修复日期 | 修复版本 |
|----|------|------|----------|----------|
| ID-1 | 添加飞机姿态参数 | 对象控制 | 2026-06-29 | v1.1 |
| ID-2 | 删除河流透明度 | 显示参数 | 2026-06-29 | v1.1 |
| ID-3 | 测距/测角/路径坐标拾取错误 | 路径规划/测量 | 2026-06-29 | v1.1 |
| ID-4 | 鼠标坐标放在相机坐标前面 | 坐标信息 | 2026-06-29 | v1.1 |
| ID-5 | 只有aircraft显示姿态参数 | 对象控制 | 2026-06-29 | v1.1 |
| ID-6 | 航向角/俯仰角/滚转角局部旋转 | 对象控制(aircraft) | 2026-06-29 | v1.2 |
| ID-7 | 全局复位+相机复位分离 | 视角/对象控制 | 2026-06-29 | v1.1 |
| ID-8 | 删除测量线(清除/撤销) | 测量工具 | 2026-06-29 | v1.1 |
| ID-9 | 树的位置放到terrain表面 | 场景构建 | 2026-06-30 | v1.2 |
| ID-10 | 保存/载入aircraft+terrain JSON数据 | 数据持久化 | 2026-06-30 | v1.2 |
| ID-11 | 路径点动态飞行 | 路径规划/飞行动画 | 2026-06-30 | v1.2 |
| ID-12 | 编程算法概要.md文件 | 文档 | 2026-06-30 | v1.2 |
| ID-13 | 保存加载数据问题 | 数据持久化 | 2026-06-30 | v1.2 |
| ID-14 | 在路径点处瞬间切换姿态（不连续） | 飞行动画 | 2026-06-30 | v1.2 |
| ID-15 | 穿过路径点的时间一致但速度不一致 | 飞行动画 | 2026-06-30 | v1.2 |
| ID-16 | 精准添加路径点（双步骤向导） | 路径规划/UI | 2026-06-30 | v1.2 |
| ID-17 | 精准添加修改 | 路径规划/UI | 2026-07-03 | v1.3 |
| ID-18 | 添加图层 | 图层管理 | 2026-07-03 | v1.3 |
| ID-19 | 精准添加再次修改 | 路径规划/UI | 2026-07-03 | v1.4 |
| ID-20 | 添加时间轴/飞机属性 | 路径规划/飞行动画 | 2026-07-03 | v1.4 |
| ID-21 | 地形颜色/植被图层/ID-3拾取升级 | 场景构建/图层管理 | 2026-07-03 | v1.5 |
| ID-22 | 删除东北天坐标输入 + UI清理 + 隐藏高程标量条 + 时间轴常驻 + FlightWindow二次修复 | 路径规划/飞行动画 | 2026-07-03 | v1.6 |
| ID-23 | FlightWindow 独立窗口崩溃 (macOS双QVTK) | 飞行动画 | 2026-07-03 | v1.7 |
| ID-24 | 添加FlightPlotter时误吞ClickablePlotter鼠标事件方法 | 3D交互 | 2026-07-03 | v1.7 |
| ID-25 | FlightWindow 独立窗口 + 相机跟随完全删除 | 飞行动画/UI | 2026-07-03 | v1.8 |
| ID-26 | 沙地/草地/土地图层默认不勾选 | 场景构建/图层管理 | 2026-07-03 | v1.8 |
| ID-27 | 增加编队功能 | 飞行动画/UI | 2026-07-06 | v1.9 |
| ID-28 | PyVista `extract_surface(algorithm=None)` 崩溃 | DEM导入/场景构建 | 2026-07-06 | v2.0 |
| ID-29 | 多项功能新增与改进（鼠标坐标精度/菜单图层/Z夸张/7000m高度/全地形覆盖/XY自动范围） | 全局 | 2026-07-06 | v2.1 |
| ID-30 | DEM飞机放大（AIRCRAFT_DEFAULT_SCALE 500→2000） | DEM导入 | 2026-07-06 | v2.2 |
| ID-31 | DEM对象控制优化（仅显示aircraft1/2/terrain，默认aircraft1） | 对象控制/UI | 2026-07-06 | v2.2 |
| ID-32 | DEM相机视图修复（俯视/侧视/复位使用地形动态范围） | 视角 | 2026-07-06 | v2.2 |
| ID-33 | DEM保存/载入修复（保存X,Y网格实现完整重构） | 数据持久化 | 2026-07-06 | v2.2 |
| ID-34 | 删除STL/OBJ导入，仅保留DEM导入 | 导入 | 2026-07-06 | v2.2 |
| ID-35 | 3D交互鲁棒性改进（picker精度提升/None坐标处理） | 3D交互 | 2026-07-06 | v2.2 |
| ID-36 | 新增图层管理对话框（XY绘图工具+矩形/圆形/多边形选区） | 图层管理/UI | 2026-07-06 | v2.2 |
| ID-37 | UI重构：场景树+属性面板+坐标简化+ASC导入导出+飞行/编队修复 | 全局 | 2026-07-07 | v2.4 |
| ID-38 | 操作日志+场景设置+路径点恢复 | 全局 | 2026-07-08 | v2.4 |
| ID-39 | macOS Rosetta 2 VTK 启动死锁（import vtk 144模块 → 4模块） | 启动/VTK | 2026-07-08 | v2.4 |
| ID-40 | Enterprise重构: 事件驱动+魔法数字+UX防御+DEM场景修复 | 全局 | 2026-07-08 | v2.5 |

