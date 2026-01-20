#include "ElementBase.h"
#include "DataStructure/Node/Node.h"

ElementBase::ElementBase()
{
}

void ElementBase::GetDOFs(std::vector<int>& DOFs)
{
    int NodeDOF = Get_NodeDOF();
    int numDOFs = m_pNode.size() * NodeDOF;
    DOFs.resize(numDOFs);

    int index = 0;
    for (auto& node : m_pNode)
    {
        auto pNode = node.lock();
        for(int i = 0; i < NodeDOF; i++)
        {
            DOFs[index++] = pNode->m_DOF[i];
        }
    }
}
