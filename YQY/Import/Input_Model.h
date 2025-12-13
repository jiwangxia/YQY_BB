#pragma once
#include "Base/Base.h"
#include <QTextStream>
#include <QFile>
//#include "Utility/EnumKeyword.h"

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
	// 读取截面数据
	bool InputSection(QTextStream& flow, const QStringList& list_str);
	// 读取材料
	bool InputMaterial(QTextStream& flow, const QStringList& list_str);
	// 读取分析步
	//bool InputStep(QTextStream& flow, const QStringList& list_str, const QString& stepType);
	// 读取荷载
	bool InputLoad(QTextStream& flow, const QStringList& list_str);
	// 读取约束
	bool InputConstraint(QTextStream& flow, const QStringList& list_str);
};

