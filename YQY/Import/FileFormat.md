# YQY CAE 输入文件格式说明

## 概述

输入文件为纯文本格式，使用关键字标识不同的数据块。

- **关键字行**：以 `*` 开头
- **注释行**：以 `**` 或 `//` 开头，会被忽略
- **数据分隔**：支持空格、Tab、逗号

---

## 关键字列表

| 关键字 | 说明 |
|--------|------|
| `*NODE` | 节点定义 |
| `*ELEMENT` | 单元定义 |
| `*MATERIAL` | 材料定义 |
| `*SECTION` | 截面定义 |
| `*CONSTRAINT` | 约束定义 |
| `*LOAD` | 荷载定义 |
| `*ANALYSIS_STEP` | 分析步定义 |
| `*SPRING` | 非线性弹簧力-相对位移行为 |

---

## 1. 节点 (*NODE)

```
*NODE, 数量
ID  X  Y  Z
```

**示例：**
```
*NODE, 3
1   0.0   0.0   0.0
2   1.0   0.0   0.0
3   2.0   0.0   0.0
```

---

## 2. 单元 (*ELEMENT)

```
*ELEMENT, 类型, 数量
ID  Node1  Node2  MaterialID  SectionID
```

**支持的单元类型：**

| 类型 | 说明 | 节点数 |
|------|------|--------|
| `T3D2` | 桁架单元 | 2 |
| `CABLE` | 索单元 | 2 |
| `B31` | 梁单元 | 2 |
| `SPRING1` | 节点对地固定方向弹簧 | 1 |
| `SPRING2` | 两节点固定方向弹簧 | 2 |
| `SPRINGA` | 两节点轴向弹簧 | 2 |

**示例：**
```
*ELEMENT, CABLE, 2
1   1   2   1   1
2   2   3   1   1
```

### 弹簧单元

弹簧行为独立于材料和截面，必须在引用它的弹簧单元之前定义。当前仅支持静力平动自由度，DOF 直接使用程序内部与荷载、约束一致的 `0`、`1`、`2` 编号，分别代表 `U1`、`U2`、`U3`，导入时不再转换。

```
*SPRING, 行为ID, 数据点数量
力  相对位移
```

力-相对位移数据按相对位移严格递增，字段顺序与 Abaqus 的 `*SPRING, NONLINEAR` 一致。

```
*ELEMENT, SPRING1, 数量
ID  节点ID  DOF  行为ID

*ELEMENT, SPRING2, 数量
ID  节点1ID  DOF1  节点2ID  DOF2  行为ID

*ELEMENT, SPRINGA, 数量
ID  节点1ID  节点2ID  行为ID
```

`SPRINGA` 不指定 DOF；它始终利用两端节点的三个平动自由度，后续静力实现中沿两节点当前连线方向作用。

---

## 3. 材料 (*MATERIAL)

```
*MATERIAL, 数量
ID  弹性模量  泊松比  密度  许用应力  膨胀系数
```

**示例：**
```
*MATERIAL, 1
1   7e+10   0.3   3836   0   1.84e+10
```

---

## 4. 截面 (*SECTION)

```
*SECTION, 数量
ID  面积
```

**示例：**
```
*SECTION, 1
1   0.01199
```

---

## 5. 约束 (*CONSTRAINT)

```
*CONSTRAINT, 数量
ID  NodeID  Direction  Value
```

**方向编码：**

| 值 | 方向 |
|----|------|
| 0 | X |
| 1 | Y |
| 2 | Z |
| 3 | RX |
| 4 | RY |
| 5 | RZ |

**示例：**
```
*CONSTRAINT, 6
1   1   0   0
2   1   1   0
3   1   2   0
4   3   0   0
5   3   1   0
6   3   2   0
```

---

## 6. 荷载 (*LOAD)

```
*LOAD, 类型, 数量
ID  NodeID/ElementID  Direction  Value
```

**支持的荷载类型：**

| 类型 | 说明 |
|------|------|
| `FORCE_NODE` | 节点力 |
| `FORCE_ELEMENT` | 单元荷载 |

**示例：**
```
*LOAD, FORCE_NODE, 2
1   2   1   -1000
2   2   2   -500
```

---

## 7. 分析步 (*ANALYSIS_STEP)

```
*ANALYSIS_STEP, 数量
ID  Type  Time  StepSize  Tolerance  MaxIterations
```

**分析类型：**

| 类型 | 说明 |
|------|------|
| `STATIC` | 静力分析 |
| `DYNAMIC` | 动力分析 |

**示例：**
```
*ANALYSIS_STEP, 1
1   STATIC   1.0   0.1   1e-5   32
```

---

## 完整示例

```
** 这是一个简单的两节点桁架模型
** 作者：YQY

*NODE, 2
1   0.0   0.0   0.0
2   1.0   0.0   0.0

*MATERIAL, 1
1   2.1e+11   0.3   7850   200e6   1.2e-5

*SECTION, 1
1   0.01

*ELEMENT, T3D2, 1
1   1   2   1   1

*CONSTRAINT, 3
1   1   0   0
2   1   1   0
3   1   2   0

*LOAD, FORCE_NODE, 1
1   2   0   10000

*ANALYSIS_STEP, 1
1   STATIC   1.0   0.1   1e-5   32
```

---

## 注意事项

> [!IMPORTANT]
> 1. 关键字必须以 `*` 开头，且紧跟关键字名称
> 2. `*ELEMENT` 和 `*LOAD` 需要指定类型作为第二个参数
> 3. 数据字段之间可以用空格、Tab 或逗号分隔
> 4. 节点/单元 ID 会在读取时自动重新编号
