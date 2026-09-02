#pragma once

#include <QtTypes>

#include <vector>

// HDF5 文件中模型和结果数据的快速摘要。
struct Hdf5ResultSummary
{
    bool hasModel = false;             // 是否包含模型数据
    bool hasResult = false;            // 是否包含结果数据
    qint64 frameCount = 0;             // 结果帧数量
    qint64 displacementRecordCount = 0; // 位移记录数量
    qint64 stressRecordCount = 0;      // 应力记录数量
    qint64 strainRecordCount = 0;      // 应变记录数量
    bool partialResult = false;        // 结果流是否未正常完成
};

// 一帧结果的索引信息。
struct Hdf5ResultFrameInfo
{
    int domainId = 0;          // 全局结果域编号
    int stepId = 0;            // 分析步编号
    int increment = 0;         // 增量步编号
    int analysis = 0;          // 分析类型编码
    double time = 0.0;         // 对应物理时间
    double loadFactor = 1.0;   // 静力荷载系数
};

// 单个节点的一帧结果。
struct Hdf5NodalResult
{
    int id = 0;                       // 节点编号
    double displacement[6] = {};      // 平动和转动位移
    double velocity[6] = {};          // 平动和转动速度
    double acceleration[6] = {};      // 平动和转动加速度
    double currentCoordinate[3] = {}; // 当前全局坐标
};

// 单个单元的一帧结果。
struct Hdf5ElementResult
{
    int id = 0;                 // 单元编号
    double axialForce = 0.0;    // 轴力
    double shearY = 0.0;        // 局部 Y 向剪力
    double shearZ = 0.0;        // 局部 Z 向剪力
    double torque = 0.0;        // 扭矩
    double momentY = 0.0;       // 局部 Y 向弯矩
    double momentZ = 0.0;       // 局部 Z 向弯矩
    double initStress = 0.0;    // 初始应力
    double currentStress = 0.0; // 当前应力
    double deltaStress = 0.0;   // 应力增量
    double strain = 0.0;        // 轴向应变
};

// 由帧索引、节点结果和单元结果组成的一帧数据。
struct Hdf5ResultFrame
{
    Hdf5ResultFrameInfo info;               // 帧索引信息
    std::vector<Hdf5NodalResult> nodes;     // 节点结果
    std::vector<Hdf5ElementResult> elements; // 单元结果
};

// 一个结果量的有效数值范围。
struct Hdf5ResultRange
{
    double minimum = 0.0; // 最小值
    double maximum = 0.0; // 最大值
    bool valid = false;   // 范围是否可用
};

// 后处理颜色映射使用的结果范围集合。
struct Hdf5ResultRanges
{
    Hdf5ResultRange displacementMagnitude; // 位移幅值范围
    Hdf5ResultRange displacementX;         // X 向位移范围
    Hdf5ResultRange displacementY;         // Y 向位移范围
    Hdf5ResultRange displacementZ;         // Z 向位移范围
    Hdf5ResultRange axialForce;            // 轴力范围
    Hdf5ResultRange stress;                // 应力范围
    Hdf5ResultRange strain;                // 应变范围
};
