#pragma once

#include <QDialog>
#include <QSet>
#include <QStringList>

#include <functional>
#include <memory>
#include <utility>

class StructureData;
class QTableWidget;

class AnalysisManagerDialog : public QDialog
{
public:
    enum class Page
    {
        Steps = 0,
        Loads = 1,
        Constraints = 2,
        MPCs = 3
    };

    explicit AnalysisManagerDialog(const std::shared_ptr<StructureData>& structure,
        Page initialPage, QWidget* parent = nullptr);

    bool modelChanged() const { return m_modelChanged; }
    void setModelChangedCallback(std::function<void(const QSet<int>&)> callback);
    void setOpenManagerCallback(std::function<void(Page)> callback);
    void refreshFromModel();

private:
    void refreshAll();
    void refreshStepTable(int preferredId = -1);
    void refreshLoadTable(int preferredId = -1);
    void refreshConstraintTable(int preferredId = -1);
    void refreshMPCTable(int preferredId = -1);
    void editStep(int stepId = -1);
    void editLoad(int loadId = -1);
    void editConstraint(int constraintId = -1);
    void editMPC(int mpcId = -1);
    void deleteSelectedStep();
    void deleteSelectedLoad();
    void deleteSelectedConstraint();
    void deleteSelectedMPC();
    QSet<int> affectedStepsFrom(int firstStepId) const;
    void markModelChanged(const QSet<int>& affectedStepIds);
    int currentId(QTableWidget* table) const;
    void selectId(QTableWidget* table, int id) const;

    std::shared_ptr<StructureData> m_structure;
    Page m_page = Page::Steps;
    QTableWidget* m_stepTable = nullptr;
    QTableWidget* m_loadTable = nullptr;
    QTableWidget* m_constraintTable = nullptr;
    QTableWidget* m_mpcTable = nullptr;
    bool m_modelChanged = false;
    std::function<void(const QSet<int>&)> m_modelChangedCallback;
    std::function<void(Page)> m_openManagerCallback;
};

class AnalysisStepManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisStepManagerDialog(const std::shared_ptr<StructureData>& structure,
        QWidget* parent = nullptr)
        : AnalysisManagerDialog(structure, Page::Steps, parent) {}
};

class AnalysisLoadManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisLoadManagerDialog(const std::shared_ptr<StructureData>& structure,
        QWidget* parent = nullptr)
        : AnalysisManagerDialog(structure, Page::Loads, parent) {}
};

class AnalysisConstraintManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisConstraintManagerDialog(const std::shared_ptr<StructureData>& structure,
        QWidget* parent = nullptr)
        : AnalysisManagerDialog(structure, Page::Constraints, parent) {}
};

class AnalysisMPCManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisMPCManagerDialog(const std::shared_ptr<StructureData>& structure,
        QWidget* parent = nullptr)
        : AnalysisManagerDialog(structure, Page::MPCs, parent) {}
};
