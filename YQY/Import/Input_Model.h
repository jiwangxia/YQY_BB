#pragma once
#include "Base/Base.h"
#include <QTextStream>
#include <QFile>
#include <functional>

class StructureData;

/**
 * @brief 模型输入类 - 负责从文件读取有限元模型数据
 */
class Input_Model : public Base
{
private:
	int g_Direction;
public:
	/**
	 * @brief 读取一行有效数据（跳过注释和空行）
	 * @param [in] flow 文本流
	 * @param [out] str 读取到的字符串
	 * @return 成功返回 true，到达文件末尾返回 false
	 */
	bool ReadLine(QTextStream& flow, QString& str);

	/**
	 * @brief 读取模型数据文件
	 * @param [in] FileName 文件路径
	 * @param [in] pStructure 结构数据对象
	 * @return 读取成功返回 true
	 */
	bool InputData(const QString& FileName, std::shared_ptr<StructureData> pStructure);

private:
	std::shared_ptr<StructureData> m_Structure;  ///< 结构数据指针

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

	/// @name 单元处理函数映射
	/// @{
	using ElementHandler = std::function<bool(Input_Model*, QTextStream&, const QStringList&, int)>;
	static const QMap<EnumKeyword::ElementType, ElementHandler> s_ElementHandlers;  ///< 单元类型到处理函数的映射表
	
	/**
	 * @brief 读取桁架单元数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nElement 单元数量
	 * @return 读取成功返回 true
	 */
	bool InputElementTruss(QTextStream& flow, const QStringList& list_str, int nElement);

	/**
	 * @brief 读取索单元数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nElement 单元数量
	 * @return 读取成功返回 true
	 */
	bool InputElementCable(QTextStream& flow, const QStringList& list_str, int nElement);

	/**
	 * @brief 读取梁单元数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @param [in] nElement 单元数量
	 * @return 读取成功返回 true
	 */
	bool InputElementBeam(QTextStream& flow, const QStringList& list_str, int nElement);
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

	/// @name 荷载处理函数映射
	/// @{
	/**
	 * @brief 读取荷载数据（分发到具体荷载处理函数）
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
	bool InputLoad(QTextStream& flow, const QStringList& list_str);

	using LoadHandler = std::function<bool(Input_Model*, QTextStream&, const QStringList&, int)>;
	static const QMap<EnumKeyword::LoadType, LoadHandler> s_LoadHandlers;  ///< 荷载类型到处理函数的映射表

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
	 * @brief 读取约束数据
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
	bool InputConstraint(QTextStream& flow, const QStringList& list_str);

	/**
	 * @brief 读取单元应力
	 * @param [in] flow 文本流
	 * @param [in] list_str 关键字行解析后的字符串列表
	 * @return 读取成功返回 true
	 */
	bool InputElement_Stress(QTextStream& flow, const QStringList& list_str);


};

