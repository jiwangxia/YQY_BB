#pragma once
#include "Base/Base.h"
#include <QTextStream>
#include <QFile>

class StructureData;
class ElementBase;

/**
 * @brief 模型输入类 - 负责从文件读取有限元模型数据
 */
class Input_Model : public Base
{
public:
    /**
	 * @brief 读取一行有效数据（跳过注释和空行）
	 * @param [in] flow 文本流
	 * @param [out] str 读取到的字符串
	 * @return 成功返回 true，到达文件末尾返回 false
	 */
    bool ReadLine(QTextStream& flow, _OUT QString& str);

    /**
	 * @brief 读取模型数据文件，成功后替换目标模型，失败时保持目标不变
	 * @param [in] FileName 文件路径
	 * @param [in] pStructure 结构数据对象
	 * @return 读取成功返回 true
	 */
    bool InputData(const QString& FileName, std::shared_ptr<StructureData> pStructure);
    QString LastError() const
    {
        return m_LastError;
    }

private:
    std::shared_ptr<StructureData> m_Structure; ///< 当前解析模型，成功提交后关联目标模型
    QString m_LastError;
    bool ValidateStructure(_OUT QString& errorMessage) const;
    bool ValidateElementGeometry(const ElementBase& element, _OUT QString& errorMessage) const;
    bool ValidateLoadsAndConstraints(_OUT QString& errorMessage) const;

    /**
	 * @brief 读取节点数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputNodes(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取单元数据（分发到具体单元处理函数）
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputElement(QTextStream& flow, const QStringList& list_str);

    /// @name 单元读取函数
    /// @{
    // 桁架、索和 CR3D 梁共用节点、材料及截面的读取流程。
    bool InputLineElements(QTextStream& flow, EnumKeyword::ElementType type, int nElement);

    bool InputElementSpring1(QTextStream& flow, const QStringList& list_str, int nElement);
    bool InputElementSpring2(QTextStream& flow, const QStringList& list_str, int nElement);
    bool InputElementSpringA(QTextStream& flow, const QStringList& list_str, int nElement);
    bool InputSpringBehavior(QTextStream& flow, const QStringList& list_str);
    bool InputAeroElements(QTextStream& flow, const QStringList& list_str);
    /// @}

    /**
	 * @brief 读取截面数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputSection(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取材料数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputMaterial(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取分析步数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputAnalysisStep(QTextStream& flow, const QStringList& list_str);

    /// @name 荷载读取函数
    /// @{
    /**
	 * @brief 读取荷载数据（分发到具体荷载处理函数）
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputLoad(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取节点力荷载数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nLoad 荷载数量
	 * @return 读取成功返回 true
	 */
    bool InputForceNode(QTextStream& flow, const QStringList& list_str, int nLoad);

    /**
	 * @brief 读取单元荷载数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nLoad 荷载数量
	 * @return 读取成功返回 true
	 */
    bool InputForceElement(QTextStream& flow, const QStringList& list_str, int nLoad);

    /**
	 * @brief 读取重力数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nLoad 荷载数量
	 * @return 读取成功返回 true
	 */
    bool InputForceGravity(QTextStream& flow, const QStringList& list_str, int nLoad);

    /**
	 * @brief 读取风荷载数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nLoad 荷载数量
	 * @return 读取成功返回 true
	 */
    bool InputForceWind(QTextStream& flow, const QStringList& list_str, int nLoad);

    /**
	 * @brief 读取约束数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputConstraint(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取主从多点约束：MPC_ID, MasterNode, SlaveNode, SlaveDirection
	 */
    bool InputMPC(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取按约束编号关联的分段线性位移时程
	 * @param [in,out] flow 模型文件输入流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputConstraintTabular(QTextStream& flow, const QStringList& list_str);

    /**
	 * @brief 读取单元应力
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
    bool InputElement_Stress(QTextStream& flow, const QStringList& list_str);
};
