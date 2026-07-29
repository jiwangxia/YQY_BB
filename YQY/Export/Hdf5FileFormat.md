# YQY H5/HDF5 文件格式说明

## 1. 设计原则

YQY 的 H5 文件用于保存模型前处理数据和计算结果数据。内部组名、数据集名和字段名统一使用英文大写，避免中文路径在不同 HDF5 工具、不同系统编码下出现乱码。需要中文显示时，建议后处理程序在界面层做翻译，或后续通过 UTF-8 属性保存中文标题。

当前文件格式分为三部分：

| 分组 | 含义 |
| --- | --- |
| `/YQY/INPUT` | 模型输入数据，也就是前处理数据。 |
| `/YQY/RESULT` | 计算结果数据。 |
| `/INDEX/YQY/RESULT` | 结果索引数据，用于定位每一帧结果在结果数据集中的位置。 |

H5 文件可以有两种写入方式：

| 写入方式 | 使用场景 | 说明 |
| --- | --- | --- |
| 一次性写入 | 静力分析 | 计算结束后，将内存中的模型和结果统一写入 H5。 |
| 流式写入 | 动力分析 | 计算前先写入模型，计算过程中每个时间步追加写入结果，避免完整历史结果长期保存在内存中。 |

## 2. 文件根属性

根对象保存文件级元信息。

| 属性名 | 类型 | 说明 |
| --- | --- | --- |
| `FORMAT` | string | 文件格式标识，当前为 `YQY_H5`。 |
| `CREATED_TIME` | string | 文件创建时间，格式为 `yyyy-MM-dd hh:mm:ss`。 |
| `PROGRAM` | string | 生成文件的程序名，当前为 `YQY_CAE`。 |
| `SOURCE_MODEL` | string | 原始模型文件名，可以为空。 |

## 3. 模型输入数据

模型输入数据保存在 `/YQY/INPUT` 下。当前 `DOMAIN_ID` 固定为 `1`，表示这些数据属于同一个模型域。

读取端将节点、材料、截面、属性、单元、单点约束、荷载和分析步视为基础数据集，缺失时判定文件不完整。后续扩展的数据集（例如 MPC、分析步元数据、模型集和计算区域）采用可选读取：存在时读取并校验，不存在时使用空集合或默认值。保存端始终按当前完整结构输出。

### 3.1 `/YQY/INPUT/NODE/GRID`

节点定义数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 节点编号。 |
| `X` | double[3] | 节点初始坐标，顺序为 `X, Y, Z`。 |
| `DOF_COUNT` | int | 节点自由度数量。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.2 `/YQY/INPUT/MATERIAL/MAT`

材料定义数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 材料编号。 |
| `E` | double | 弹性模量。 |
| `NU` | double | 泊松比。 |
| `RHO` | double | 密度。 |
| `MAX_STRESS` | double | 初始屈服应力。字段名为兼容旧结果文件而保留。 |
| `EXPANSION` | double | 热膨胀系数。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.3 `/YQY/INPUT/SECTION/SECTION`

截面定义数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 截面编号。 |
| `TYPE` | int | 截面类型枚举值，见“枚举值说明”。 |
| `AREA` | double | 截面面积。 |
| `RADIUS` | double | 圆形截面半径。非圆形截面可为 `0`。 |
| `WIDTH` | double | 矩形截面宽度。非矩形截面可为 `0`。 |
| `HEIGHT` | double | 矩形截面高度。非矩形截面可为 `0`。 |
| `IY` | double | 截面绕局部 y 轴惯性矩。 |
| `IZ` | double | 截面绕局部 z 轴惯性矩。 |
| `J` | double | 截面扭转常数。当前未完整计算时可为 `0`。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.4 `/YQY/INPUT/PROPERTY/PROPERTY`

属性定义数据，用于连接材料和截面。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 属性编号。 |
| `MID` | int | 材料编号，对应 `/YQY/INPUT/MATERIAL/MAT` 中的 `ID`。 |
| `SID` | int | 截面编号，对应 `/YQY/INPUT/SECTION/SECTION` 中的 `ID`。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.5 `/YQY/INPUT/ELEMENT/ELEMENT`

单元定义数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `EID` | int | 单元编号。 |
| `TYPE` | int | 单元类型枚举值，见“枚举值说明”。 |
| `PID` | int | 属性编号，对应 `/YQY/INPUT/PROPERTY/PROPERTY` 中的 `ID`。 |
| `G` | int[2] | 单元连接节点编号。当前输出接口按二节点单元保存。 |
| `Q0` | double[3] | 梁单元初始方向向量。非梁单元可为 `0, 0, 0`。 |
| `INIT_STRESS` | double | 单元初始应力。 |
| `ROLE` | int | 单元业务用途：普通、导线、耐张硬件、相内间隔棒、相间间隔棒或悬垂硬件。 |
| `WIRE_ID` | int | 子导线编号；非导线单元为 `-1`。 |
| `AERO_PROFILE_ID` | int | 气动参数编号；小于 `0` 表示不参与气动计算。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.6 `/YQY/INPUT/CONSTRAINT/SPC`

单点约束数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 约束编号。 |
| `G` | int | 被约束节点编号。 |
| `C` | int | 约束方向枚举值，见“枚举值说明”。 |
| `D` | double | 约束值。通常为 `0`。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

#### 3.6.1 `/YQY/INPUT/CONSTRAINT/MPC`

多点约束数据。当前保存端始终输出该数据集；没有 MPC 时包含零条记录。读取端允许该数据集缺失，缺失时按“没有 MPC 约束”处理。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | MPC 编号。 |
| `RELATION_TYPE` | int | 约束关系类型：`0` 为定距、`1` 为刚性偏置、`2` 为平面剪切释放。 |
| `MASTER_G` | int | 主节点编号。 |
| `SLAVE_G` | int | 从节点编号。 |
| `SLAVE_DIRECTION_MASK` | int | 从节点受约束自由度位掩码。 |
| `STEP_ID` | int | 所属分析步编号。 |
| `PARAMETERS` | double[3] | 类型相关参数：定距使用第一个值，刚性偏置使用三个分量。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.7 `/YQY/INPUT/LOAD/LOAD`

荷载定义数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 荷载编号。 |
| `TYPE` | int | 荷载类型枚举值，见“枚举值说明”。 |
| `TARGET_ID` | int | 荷载作用对象编号。节点荷载为节点编号，单元荷载为单元编号。 |
| `DIRECTION` | int | 荷载方向枚举值，见“枚举值说明”。 |
| `STEP_ID` | int | 荷载所属分析步编号。 |
| `FUNCTION_TYPE` | int | 时间函数类型枚举值。 |
| `VALUE` | double | 荷载基准值。 |
| `START_TIME` | double | 荷载开始时间。 |
| `END_TIME` | double | 荷载结束时间。 |
| `PARAMS` | double[8] | 时间函数参数，顺序为 `AMPLITUDE, FREQUENCY, PHASE, OFFSET, RAMP_T0, RAMP_T1, DECAY, PERIOD`。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.8 `/YQY/INPUT/ANALYSIS_STEP/STEP`

分析步定义数据。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 分析步编号。 |
| `TYPE` | int | 分析步类型枚举值，见“枚举值说明”。 |
| `IS_DYNAMIC` | int | 是否为动力分析，`1` 表示动力，`0` 表示非动力。 |
| `DYNAMIC_SOLVER_TYPE` | int | 动力求解器类型枚举值。 |
| `TIME` | double | 分析步总时间。 |
| `STEP_SIZE` | double | 时间步长或增量步长。 |
| `TOLERANCE` | double | 收敛容差。 |
| `MAX_ITERATIONS` | int | 最大迭代次数。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.9 `/YQY/INPUT/AERO/CASE`

气动参数工况索引数据。该数据集用于记录每一组气动参数对应的名称、来源文件和基本采样信息。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 气动工况编号。 |
| `BUNDLE_COUNT` | int | 导线分裂数。 |
| `WIND_SPEED` | int | 风速，单位为 m/s。 |
| `ICE_THICKNESS` | int | 覆冰厚度，单位为 mm。 |
| `MODEL_COUNT` | int | 当前工况下包含的气动模型数量。 |
| `DATA_SIZE` | int | 每个气动模型的角度采样点数量。 |
| `START_ANGLE` | double | 起始攻角，当前为 `0` 度。 |
| `ANGLE_STEP` | double | 攻角步长，当前为 `5` 度。 |
| `SOURCE_FILE` | string | 当前工况对应的气动参数文件名。该字段使用 UTF-8 保存，可以包含中文。 |
| `SOURCE_PATH` | string | 当前工况对应的气动参数文件路径。该字段使用 UTF-8 保存，可以包含中文。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

### 3.10 `/YQY/INPUT/AERO/COEFFICIENT`

气动力系数表数据。该数据集保存程序实际读取后的升力、阻力和力矩系数，因此即使原始气动参数文件被移动或删除，H5 文件中仍然保留计算时使用的气动参数。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `CASE_ID` | int | 气动工况编号，对应 `/YQY/INPUT/AERO/CASE` 中的 `ID`。 |
| `MODEL_INDEX` | int | 当前工况下的气动模型序号，从 `0` 开始。 |
| `ANGLE_INDEX` | int | 攻角采样点序号，从 `0` 开始。 |
| `ANGLE` | double | 攻角，单位为度。 |
| `CL` | double | 升力系数。 |
| `CD` | double | 阻力系数。 |
| `CM` | double | 力矩系数。 |
| `DOMAIN_ID` | int | 模型域编号，当前固定为 `1`。 |

## 4. 结果域数据

### 4.1 `/YQY/RESULT/DOMAINS`

结果域用于描述每一帧结果对应的分析步、增量步和时间。其他结果数据集都通过 `DOMAIN_ID` 与这里的 `ID` 对应。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 结果域编号，也可以理解为结果帧编号。 |
| `STEP_ID` | int | 分析步编号。 |
| `INCREMENT` | int | 增量步编号或时间步编号。 |
| `ANALYSIS` | int | 分析类型枚举值，通常对应静力或动力分析。 |
| `TIME` | double | 当前结果帧对应的分析时间。 |
| `LOAD_FACTOR` | double | 当前载荷因子。当前默认写入 `1.0`。 |

## 5. 节点结果数据

节点结果数据保存在 `/YQY/RESULT/NODAL` 下。各节点结果数据集都使用同一种记录格式。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `ID` | int | 节点编号。 |
| `X` | double | 第一个分量。含义由所在数据集决定。 |
| `Y` | double | 第二个分量。含义由所在数据集决定。 |
| `Z` | double | 第三个分量。含义由所在数据集决定。 |
| `RX` | double | 绕 X 方向转动分量。只在位移数据集中表示转角，其他节点结果中通常为 `0`。 |
| `RY` | double | 绕 Y 方向转动分量。只在位移数据集中表示转角，其他节点结果中通常为 `0`。 |
| `RZ` | double | 绕 Z 方向转动分量。只在位移数据集中表示转角，其他节点结果中通常为 `0`。 |
| `DOMAIN_ID` | int | 结果域编号，对应 `/YQY/RESULT/DOMAINS` 中的 `ID`。 |

### 5.1 `/YQY/RESULT/NODAL/DISPLACEMENT`

节点位移和转角结果。

| 字段 | 对应结果 |
| --- | --- |
| `X` | `U1`，X 向位移。 |
| `Y` | `U2`，Y 向位移。 |
| `Z` | `U3`，Z 向位移。 |
| `RX` | `UR1`，绕 X 轴转角。 |
| `RY` | `UR2`，绕 Y 轴转角。 |
| `RZ` | `UR3`，绕 Z 轴转角。 |

### 5.2 `/YQY/RESULT/NODAL/CURRENT_COORDINATE`

节点当前坐标结果。

| 字段 | 对应结果 |
| --- | --- |
| `X` | `CX`，当前 X 坐标。 |
| `Y` | `CY`，当前 Y 坐标。 |
| `Z` | `CZ`，当前 Z 坐标。 |

### 5.3 `/YQY/RESULT/NODAL/VELOCITY`

节点速度结果。

仅动力分析帧写入记录；静力分析帧在对应索引中的 `LENGTH` 为 `0`。

| 字段 | 对应结果 |
| --- | --- |
| `X` | `V1`，X 向速度。 |
| `Y` | `V2`，Y 向速度。 |
| `Z` | `V3`，Z 向速度。 |

### 5.4 `/YQY/RESULT/NODAL/ACCELERATION`

节点加速度结果。

仅动力分析帧写入记录；静力分析帧在对应索引中的 `LENGTH` 为 `0`。

| 字段 | 对应结果 |
| --- | --- |
| `X` | `A1`，X 向加速度。 |
| `Y` | `A2`，Y 向加速度。 |
| `Z` | `A3`，Z 向加速度。 |

### 5.5 `/YQY/RESULT/NODAL/REACTION_FORCE`

节点反力结果。

| 字段 | 对应结果 |
| --- | --- |
| `X` | `R1`，X 向反力。 |
| `Y` | `R2`，Y 向反力。 |
| `Z` | `R3`，Z 向反力。 |

## 6. 单元结果数据

单元结果数据保存在 `/YQY/RESULT/ELEMENTAL` 下。

### 6.1 `/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE`

梁单元以及未知单元类型的完整内力结果。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `EID` | int | 单元编号。 |
| `AXIAL` | double | 轴力。 |
| `SHEARY` | double | 局部 y 向剪力。 |
| `SHEARZ` | double | 局部 z 向剪力。 |
| `TORQUE` | double | 扭矩。 |
| `MY` | double | 绕局部 y 轴弯矩。 |
| `MZ` | double | 绕局部 z 轴弯矩。 |
| `DOMAIN_ID` | int | 结果域编号，对应 `/YQY/RESULT/DOMAINS` 中的 `ID`。 |

### 6.2 `/YQY/RESULT/ELEMENTAL/TRUSS_FORCE`

桁架单元的紧凑内力结果。桁架只承受轴力，不保存剪力、扭矩和弯矩。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `EID` | int | 单元编号。 |
| `DOMAIN_ID` | int | 结果域编号。 |
| `AXIAL` | double | 轴力。 |

### 6.3 `/YQY/RESULT/ELEMENTAL/CABLE_FORCE`

索单元的紧凑内力结果。保留轴力和扭矩，不保存剪力及弯矩。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `EID` | int | 单元编号。 |
| `DOMAIN_ID` | int | 结果域编号。 |
| `AXIAL` | double | 轴力。 |
| `TORQUE` | double | 扭矩；当前索单元尚未计算扭转时为 `0`，后续可直接写入。 |

### 6.4 `/YQY/RESULT/ELEMENTAL/STRESS`

单元应力结果。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `EID` | int | 单元编号。 |
| `S0` | double | 初始应力。 |
| `S` | double | 当前应力。 |
| `DS` | double | 增量应力，通常为当前应力相对初始应力或上一步应力的变化量，具体含义由求解器写入逻辑决定。 |
| `DOMAIN_ID` | int | 结果域编号，对应 `/YQY/RESULT/DOMAINS` 中的 `ID`。 |

### 6.5 `/YQY/RESULT/ELEMENTAL/STRAIN`

单元应变结果。

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `EID` | int | 单元编号。 |
| `STRAIN` | double | 单元应变。 |
| `DOMAIN_ID` | int | 结果域编号，对应 `/YQY/RESULT/DOMAINS` 中的 `ID`。 |

## 7. 结果索引数据

结果数据集按帧连续追加保存。为了避免读取某一帧时扫描整个结果数据集，`/INDEX/YQY/RESULT` 下为每一个结果数据集保存索引。

每个索引数据集的路径与对应结果数据集路径保持一致，只是在前面增加 `/INDEX`。例如：

| 结果数据集 | 索引数据集 |
| --- | --- |
| `/YQY/RESULT/NODAL/DISPLACEMENT` | `/INDEX/YQY/RESULT/NODAL/DISPLACEMENT` |
| `/YQY/RESULT/ELEMENTAL/TRUSS_FORCE` | `/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE` |
| `/YQY/RESULT/ELEMENTAL/CABLE_FORCE` | `/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE` |
| `/YQY/RESULT/ELEMENTAL/STRESS` | `/INDEX/YQY/RESULT/ELEMENTAL/STRESS` |

索引记录字段如下：

| 字段名 | 类型 | 说明 |
| --- | --- | --- |
| `DOMAIN_ID` | int | 结果域编号，对应 `/YQY/RESULT/DOMAINS` 中的 `ID`。 |
| `POSITION` | long long | 该帧数据在对应结果数据集中的起始位置。 |
| `LENGTH` | long long | 该帧数据条数。节点结果通常等于该帧节点输出数量，单元结果通常等于该帧单元输出数量。 |

读取某一帧结果时，推荐流程如下：

1. 从 `/YQY/RESULT/DOMAINS` 找到目标 `ID`。
2. 从对应 `/INDEX/...` 数据集中找到同一 `DOMAIN_ID` 的 `POSITION` 和 `LENGTH`。
3. 对结果数据集做块读取，读取区间为 `[POSITION, POSITION + LENGTH)`。

## 8. 枚举值说明

当前 H5 中枚举值直接使用 C++ 枚举的整数值。

### 8.1 方向枚举 `Direction`

| 字符串 | 当前整数值 | 说明 |
| --- | --- | --- |
| `X` | 0 | X 方向平移。 |
| `Y` | 1 | Y 方向平移。 |
| `Z` | 2 | Z 方向平移。 |
| `RX` | 3 | 绕 X 轴转动。 |
| `RY` | 4 | 绕 Y 轴转动。 |
| `RZ` | 5 | 绕 Z 轴转动。 |
| `UNKNOWN` | 6 | 未知方向。 |

### 8.2 单元类型枚举 `ElementType`

| 字符串 | 当前整数值 | 说明 |
| --- | --- | --- |
| `UNKNOWN` | 0 | 未知单元。 |
| `T3D2` | 1 | 二节点桁架单元。 |
| `CABLE` | 2 | 二节点索单元。 |
| `CR3D` | 3 | 三维 CR 梁单元。 |

### 8.3 截面类型枚举 `SectionType`

| 字符串 | 当前整数值 | 说明 |
| --- | --- | --- |
| `UNKNOWN` | 0 | 未知截面。 |
| `CIRCULAR` | 1 | 圆形截面。 |
| `L` | 2 | L 形截面。 |
| `RECTANGULAR` | 3 | 矩形截面。 |

### 8.4 荷载类型枚举 `LoadType`

| 字符串 | 当前整数值 | 说明 |
| --- | --- | --- |
| `FORCE_NODE` | 0 | 节点力。 |
| `FORCE_ELEMENT` | 1 | 单元荷载。 |
| `FORCE_GRAVITY` | 2 | 重力荷载。 |
| `FORCE_WIND` | 3 | 风荷载。 |
| `FORCE_ICE` | 4 | 覆冰荷载。 |
| `UNKNOWN` | 5 | 未知荷载。 |

### 8.5 分析步类型枚举 `StepType`

| 字符串 | 当前整数值 | 说明 |
| --- | --- | --- |
| `STATIC` | 0 | 静力分析。 |
| `DYNAMIC` | 1 | 动力分析。 |
| `UNKNOWN` | 2 | 未知分析类型。 |

## 9. H5 输出策略

H5 是求解的必需输出，不再由 BDF 关键字控制。默认输出到
`Export/ExportH5/<输入模型名>.h5`；GUI 算例任务会为每个算例生成独立文件名。

| 分析类型 | 保存策略 |
| --- | --- |
| 静力分析 | 计算完成后统一写入 H5。 |
| 动力分析 | 计算过程中按时间步流式写入 H5。 |

## 10. 后续扩展建议

1. 增加中文显示属性，例如 `TITLE_CN`、`COMPONENTS_CN`，但内部路径和字段名仍保持英文。
2. 增加单位属性，例如 `UNIT` 或 `COMPONENT_UNITS`，便于后处理显示。
3. 增加模型拓扑元信息，例如节点集、单元集、坐标系、结果请求列表。
4. 增加枚举映射数据集，避免后续 C++ 枚举顺序改变后旧文件解释错误。
