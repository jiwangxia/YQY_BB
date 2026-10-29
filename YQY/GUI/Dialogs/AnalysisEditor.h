#pragma once

#include "Utility/EnumKeyword.h"

#include <QDialog>
#include <array>
#include <memory>

class AnalysisStep;
struct AnalysisStepConfig;
class Constraint;
class LoadBase;
class Node;
class NonlinearMPCConstraint;
class StructureData;

/** @brief 分析步编辑器的草稿读取接口，确认后由管理器写回模型。 */
class AnalysisStepEditor : public QDialog
{
public:
    using QDialog::QDialog;
    virtual AnalysisStepConfig config() const = 0;                                                // 读取当前编辑的分析步配置
    virtual bool shapePreservingAerodynamicInterpolation() const = 0;                             // 读取气动系数是否使用保形插值
    virtual std::array<int, 4> quadWireProfiles() const = 0;                                      // 读取四分裂导线各子线的气动曲线绑定
};

/** @brief 荷载编辑器的已确认结果接口，取消时不写回模型。 */
class AnalysisLoadEditor : public QDialog
{
public:
    using QDialog::QDialog;
    virtual std::shared_ptr<LoadBase> value() const = 0;                                          // 返回确认后的荷载对象
};

/** @brief 位移约束编辑器的结果接口，节点有效性在确认时检查。 */
class AnalysisConstraintEditor : public QDialog
{
public:
    using QDialog::QDialog;
    virtual QString name() const = 0;                                                             // 读取约束名称
    virtual int stepId() const = 0;                                                               // 读取约束首次生效的分析步编号
    virtual std::shared_ptr<Node> node() const = 0;                                               // 返回校验通过的约束节点
    virtual EnumKeyword::Direction direction() const = 0;                                         // 读取约束自由度方向
    virtual double displacement() const = 0;                                                      // 读取给定的约束位移值
};

/** @brief MPC 编辑器的已确认结果接口，确认时检查主从节点和方向。 */
class AnalysisMPCEditor : public QDialog
{
public:
    using QDialog::QDialog;
    virtual std::shared_ptr<NonlinearMPCConstraint> value() const = 0;                            // 返回确认后的 MPC 约束对象
};

class AnalysisEditorFactory final
{
public:
    /** @brief 创建分析步编辑器，复用正式界面的草稿和确认流程。
     *
     * @param id         分析步编号
     * @param structure  编辑上下文中的结构模型
     * @param existing   已有分析步；为空时创建新分析步草稿
     * @param parent     父窗口
     * @return 由调用方拥有的编辑器，确认前不修改结构模型。
     * @author   yangqiuyou
     * @date     2026.10.16
     */
    static std::unique_ptr<AnalysisStepEditor> CreateStep(int id, const std::shared_ptr<StructureData>& structure,
                                                          const std::shared_ptr<AnalysisStep>& existing,
                                                          QWidget* parent = nullptr);             // 创建分析步草稿编辑器

    /** @brief 创建荷载编辑器，复用正式界面的目标和时间函数校验。
     *
     * @param id         荷载编号
     * @param structure  编辑上下文中的结构模型
     * @param existing   已有荷载；为空时创建新荷载草稿
     * @param parent     父窗口
     * @return 由调用方拥有的编辑器，确认后通过 value() 读取结果。
     * @author   yangqiuyou
     * @date     2026.10.16
     */
    static std::unique_ptr<AnalysisLoadEditor> CreateLoad(int id, const std::shared_ptr<StructureData>& structure,
                                                          const std::shared_ptr<LoadBase>& existing,
                                                          QWidget* parent = nullptr);             // 创建荷载编辑器并保留确认校验流程

    /** @brief 创建位移约束编辑器，复用正式界面的节点校验。
     *
     * @param id         约束编号
     * @param structure  编辑上下文中的结构模型
     * @param existing   已有约束；为空时创建新约束草稿
     * @param parent     父窗口
     * @return 由调用方拥有的编辑器，确认后读取约束各字段。
     * @author   yangqiuyou
     * @date     2026.10.16
     */
    static std::unique_ptr<AnalysisConstraintEditor> CreateConstraint(int id,
                                                                      const std::shared_ptr<StructureData>& structure,
                                                                      const std::shared_ptr<Constraint>& existing,
                                                                      QWidget* parent = nullptr); // 创建位移约束编辑器

    /** @brief 创建 MPC 编辑器，复用正式界面的主从关系和冲突校验。
     *
     * @param id         MPC 编号
     * @param structure  编辑上下文中的结构模型
     * @param existing   已有 MPC；为空时创建新 MPC 草稿
     * @param parent     父窗口
     * @return 由调用方拥有的编辑器，确认后通过 value() 读取结果。
     * @author   yangqiuyou
     * @date     2026.10.16
     */
    static std::unique_ptr<AnalysisMPCEditor> CreateMPC(int id, const std::shared_ptr<StructureData>& structure,
                                                        const std::shared_ptr<NonlinearMPCConstraint>& existing,
                                                        QWidget* parent = nullptr);               // 创建主从节点关系编辑器
};
