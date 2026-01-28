#pragma once
#include "Base/Base.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Node/Node.h"

/**
 * @brief 单元基类 - 所有单元类型的公共基类
 */
class ElementBase : public Base
{
public:
    ElementBase();
    QVector<std::weak_ptr<Node>> m_pNode;       //节点指针数组
    std::weak_ptr<Property>      m_pProperty;   //所属属性（材料+截面）
    double m_Stress = 0.0;                      //单元应力

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 节点自由度个数
     */
    virtual int Get_NodeDOF() const = 0;

    /**
     * @brief 获取单元所有自由度编号
     * @param [out] DOFs 自由度编号数组
     */
    void GetDOFs(std::vector<int>& DOFs);

    double L0;  ///< 单元初始长度
    Eigen::VectorXd m_inforce;
    /**
     * @brief 获取单元刚度矩阵
     * @param [out] ke 单元刚度矩阵
     */
    virtual void Get_ke(MatrixXd& ke) = 0;
    virtual void Get_ke_non(MatrixXd& ke) = 0;
    virtual void Get_L0() = 0;
};
