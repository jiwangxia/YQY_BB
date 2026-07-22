#pragma once

#include <memory>
#include <QWidget>
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QSpinBox;
class StructureData;
namespace Conductor
{
class PropertyLibrary;
}
namespace Ui
{
class ConductorModuleClass;
}
class ConductorModule final : public QWidget
{
public:
    struct BuildResult
    {
        std::shared_ptr<StructureData> structure;
        QString filePath;
        QString error;
        int nodeCount = 0;
        int elementCount = 0;

        bool succeeded() const { return structure && !filePath.isEmpty(); }
    };

    explicit ConductorModule(QWidget* p = nullptr);
    ~ConductorModule() override;
    void setPropertyLibrary(Conductor::PropertyLibrary* library);
    BuildResult buildModel(Conductor::PropertyLibrary& library, const QString& generatedDirectory) const;
    QPushButton* viewLibraryButton() const;
    QLineEdit* nameEdit() const;
    QComboBox* materialCombo() const;
    QComboBox* sectionCombo() const;
    QComboBox* elementCombo() const;
    QDoubleSpinBox* startX() const;
    QDoubleSpinBox* startY() const;
    QDoubleSpinBox* startZ() const;
    QDoubleSpinBox* endX() const;
    QDoubleSpinBox* endY() const;
    QDoubleSpinBox* endZ() const;
    QComboBox* bundleCombo() const;
    QDoubleSpinBox* spacingSpin() const;
    QSpinBox* segmentsSpin() const;
    QDoubleSpinBox* stressSpin() const;
    QCheckBox* analysisCheck() const;
    QPushButton* createButton() const;

private:
    Ui::ConductorModuleClass* m_ui = nullptr;
};
