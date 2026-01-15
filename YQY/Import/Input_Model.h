#pragma once
#include "Base/Base.h"
#include <QTextStream>
#include <QFile>
#include <functional>

class StructureData;
class Input_Model : public Base
{
public:
	bool ReadLine(QTextStream& flow, QString& str);
	bool InputData(const QString& FileName, std::shared_ptr<StructureData> pStructure);

private:
	std::shared_ptr<StructureData> m_Structure;



	// 读取节点数据
	bool InputNodes(QTextStream& flow, const QStringList& list_str);
	// 读取单元数据
	bool InputElement(QTextStream& flow, const QStringList& list_str);
	
	// 单元处理函数类型定义
	using ElementHandler = std::function<bool(Input_Model*, QTextStream&, const QStringList&, int)>;
	// 单元处理函数映射表
	static const QMap<EnumKeyword::ElementType, ElementHandler> s_ElementHandlers;

	// 各类单元的具体处理函数
	bool InputElementTruss(QTextStream& flow, const QStringList& list_str, int nElement);
	bool InputElementCable(QTextStream& flow, const QStringList& list_str, int nElement);
	bool InputElementBeam(QTextStream& flow, const QStringList& list_str, int nElement);
	
	// 读取截面数据
	bool InputSection(QTextStream& flow, const QStringList& list_str);
	// 读取材料
	bool InputMaterial(QTextStream& flow, const QStringList& list_str);
	// 读取分析步
	bool InputAnalysisStep(QTextStream& flow, const QStringList& list_str);


	// 读取荷载
	bool InputLoad(QTextStream& flow, const QStringList& list_str);

	// 荷载处理函数类型定义
	using LoadHandler = std::function<bool(Input_Model*, QTextStream&, const QStringList&, int)>;
	// 荷载处理函数映射表
	static const QMap<EnumKeyword::LoadType, LoadHandler> s_LoadHandlers;

	// 各类荷载的具体处理函数
	bool InputForceNode(QTextStream& flow, const QStringList& list_str, int nLoad);
	bool InputForceElement(QTextStream& flow, const QStringList& list_str, int nLoad);


	// 读取约束
	bool InputConstraint(QTextStream& flow, const QStringList& list_str);
};

