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

**示例：**
```
*ELEMENT, CABLE, 2
1   1   2   1   1
2   2   3   1   1
```

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
ID  半径
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
