#include "AnalysisManagerDialog.h"
#include "Widgets/TableAppearance.h"

#include "DataStructure/Structure/StructureData.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include "ui_AnalysisManagerDialog.h"
#include "ui_StepEditorDialog.h"
#include "ui_LoadEditorDialog.h"
#include "ui_ConstraintEditorDialog.h"

#include <QDialogButtonBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

#include <limits>

namespace
{
enum class ManagerGlyph { Add, Edit, Delete, Steps, Load, Constraint };

QIcon managerIcon(ManagerGlyph glyph, const QPalette& palette)
{
    QPixmap pixmap(48, 48);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(3.0, 3.0);
    const QColor foreground = palette.color(QPalette::Text);
    const QColor accent = palette.color(QPalette::Highlight);
    painter.setPen(QPen(foreground, 1.35, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(Qt::NoBrush);
    switch (glyph)
    {
    case ManagerGlyph::Add:
        painter.drawEllipse(QPointF(8, 8), 5.5, 5.5);
        painter.setPen(QPen(accent, 1.7, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(8, 4.8), QPointF(8, 11.2));
        painter.drawLine(QPointF(4.8, 8), QPointF(11.2, 8));
        break;
    case ManagerGlyph::Edit:
        painter.drawRoundedRect(QRectF(2.5, 3, 8.5, 10), 1.2, 1.2);
        painter.setPen(QPen(accent, 1.7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(QPointF(6, 11), QPointF(13.2, 3.8));
        painter.drawLine(QPointF(11.6, 3.4), QPointF(13.6, 5.4));
        break;
    case ManagerGlyph::Delete:
        painter.drawRoundedRect(QRectF(4, 5, 8, 8.5), 1, 1);
        painter.drawLine(QPointF(3, 4), QPointF(13, 4));
        painter.drawLine(QPointF(6, 2.5), QPointF(10, 2.5));
        painter.setPen(QPen(accent, 1.35, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(6.5, 7), QPointF(6.5, 11));
        painter.drawLine(QPointF(9.5, 7), QPointF(9.5, 11));
        break;
    case ManagerGlyph::Steps:
        painter.drawPolyline(QPolygonF{ QPointF(2, 12), QPointF(5, 12), QPointF(5, 8),
            QPointF(9, 8), QPointF(9, 4), QPointF(14, 4) });
        painter.setBrush(accent);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(5, 12), 1.25, 1.25);
        painter.drawEllipse(QPointF(9, 8), 1.25, 1.25);
        painter.drawEllipse(QPointF(14, 4), 1.25, 1.25);
        break;
    case ManagerGlyph::Load:
        painter.drawLine(QPointF(8, 1.5), QPointF(8, 10));
        painter.drawLine(QPointF(5, 7), QPointF(8, 10));
        painter.drawLine(QPointF(11, 7), QPointF(8, 10));
        painter.setPen(QPen(accent, 1.4, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(3, 13), QPointF(13, 13));
        break;
    case ManagerGlyph::Constraint:
        painter.drawLine(QPointF(3, 2), QPointF(3, 14));
        painter.drawLine(QPointF(3, 5), QPointF(8, 5));
        painter.drawLine(QPointF(3, 11), QPointF(8, 11));
        painter.setPen(QPen(accent, 1.5, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(8, 3), QPointF(8, 13));
        painter.drawLine(QPointF(11, 3), QPointF(11, 13));
        break;
    }
    return QIcon(pixmap);
}

template <typename Map>
int nextId(const Map& values)
{
    return values.empty() ? 1 : values.rbegin()->first + 1;
}

template <typename Resource>
bool isEffectiveAtStep(const std::shared_ptr<Resource>& resource, int stepId)
{
    // StepId=0 表示初始/全局资源；更早分析步的资源也会被后续分析步继承。
    return resource && resource->m_StepId <= stepId;
}

QString stepDisplayName(int id, const std::shared_ptr<AnalysisStep>& step)
{
    return step && !step->m_Name.trimmed().isEmpty()
        ? step->m_Name.trimmed() : QStringLiteral("Step-%1").arg(id);
}

class StepEditorDialog final : public QDialog
{
public:
    explicit StepEditorDialog(int id, const std::shared_ptr<AnalysisStep>& step, QWidget* parent)
        : QDialog(parent)
        , m_id(id)
    {
        Ui::StepEditorDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("analysisStepEditorDialog"));
        setWindowTitle(step ? QStringLiteral("编辑分析步") : QStringLiteral("新增分析步"));
        setModal(true);
        m_name = form.nameEdit;
        m_name->setPlaceholderText(QStringLiteral("Step-%1").arg(id));
        m_type = form.typeCombo;
        m_type->setItemData(0, static_cast<int>(EnumKeyword::StepType::STATIC));
        m_type->setItemData(1, static_cast<int>(EnumKeyword::StepType::DYNAMIC));
        m_time = form.timeSpin;
        m_increment = form.incrementSpin;
        m_tolerance = form.toleranceSpin;
        m_iterations = form.iterationsSpin;
        m_time->setRange(1.0e-12, 1.0e12);
        m_increment->setRange(1.0e-12, 1.0e12);
        m_tolerance->setRange(1.0e-14, 1.0);
        auto* buttons = form.buttonBox;
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        if (step)
        {
            m_name->setText(step->m_Name);
            m_type->setCurrentIndex(qMax(0, m_type->findData(static_cast<int>(step->m_Type))));
            m_time->setValue(step->m_Time);
            m_increment->setValue(step->m_StepSize);
            m_tolerance->setValue(step->m_Tolerance);
            m_iterations->setValue(step->m_MaxIterations);
        }
        else
        {
            m_time->setValue(1.0);
            m_increment->setValue(0.01);
            m_tolerance->setValue(1.0e-5);
            m_iterations->setValue(50);
        }
    }

    AnalysisStepConfig config() const
    {
        AnalysisStepConfig value;
        value.id = m_id;
        value.name = m_name->text().trimmed();
        value.type = static_cast<EnumKeyword::StepType>(m_type->currentData().toInt());
        value.totalTime = m_time->value();
        value.stepSize = m_increment->value();
        value.tolerance = m_tolerance->value();
        value.maxIterations = m_iterations->value();
        return value;
    }

private:
    int m_id = 0;
    QLineEdit* m_name = nullptr;
    QComboBox* m_type = nullptr;
    QDoubleSpinBox* m_time = nullptr;
    QDoubleSpinBox* m_increment = nullptr;
    QDoubleSpinBox* m_tolerance = nullptr;
    QSpinBox* m_iterations = nullptr;
};

QString directionName(EnumKeyword::Direction direction)
{
    return EnumKeyword::MapDirection.key(direction, QStringLiteral("--"));
}

QString loadTypeName(EnumKeyword::LoadType type)
{
    switch (type)
    {
    case EnumKeyword::LoadType::FORCE_NODE: return QStringLiteral("节点力");
    case EnumKeyword::LoadType::FORCE_ELEMENT: return QStringLiteral("单元荷载");
    case EnumKeyword::LoadType::FORCE_GRAVITY: return QStringLiteral("重力");
    case EnumKeyword::LoadType::FORCE_WIND: return QStringLiteral("风荷载");
    default: return QStringLiteral("未知");
    }
}

QString loadDisplayName(int id, const std::shared_ptr<LoadBase>& load)
{
    return load && !load->m_Name.trimmed().isEmpty()
        ? load->m_Name.trimmed() : QStringLiteral("Load-%1").arg(id);
}

int loadTargetId(const std::shared_ptr<LoadBase>& load)
{
    if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load))
    {
        const auto node = nodeLoad->m_pNode.lock();
        return node ? node->m_Id : -1;
    }
    if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(load))
    {
        const auto element = elementLoad->m_pElement.lock();
        return element ? element->m_Id : -1;
    }
    return -1;
}

double loadValue(const std::shared_ptr<LoadBase>& load)
{
    if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load))
        return nodeLoad->m_Value;
    if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(load))
        return elementLoad->m_Value;
    if (const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(load))
        return gravity->m_g;
    if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(load))
        return wind->m_velocity;
    return 0.0;
}

class LoadEditorDialog final : public QDialog
{
public:
    LoadEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
        const std::shared_ptr<LoadBase>& load, QWidget* parent)
        : QDialog(parent)
        , m_id(id)
        , m_structure(structure)
        , m_original(load)
    {
        Ui::LoadEditorDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("analysisLoadEditorDialog"));
        setWindowTitle(load ? QStringLiteral("编辑荷载") : QStringLiteral("新增荷载"));
        setModal(true);
        m_name = form.nameEdit;
        m_name->setPlaceholderText(QStringLiteral("Load-%1").arg(id));
        m_step = form.stepCombo;
        m_step->addItem(QStringLiteral("初始 / 全局"), 0);
        for (const auto& [stepId, step] : structure->m_AnalysisStep)
            if (step)
                m_step->addItem(stepDisplayName(stepId, step), stepId);
        m_type = form.typeCombo;
        m_type->setItemData(0, static_cast<int>(EnumKeyword::LoadType::FORCE_NODE));
        m_type->setItemData(1, static_cast<int>(EnumKeyword::LoadType::FORCE_ELEMENT));
        m_type->setItemData(2, static_cast<int>(EnumKeyword::LoadType::FORCE_GRAVITY));
        m_type->setItemData(3, static_cast<int>(EnumKeyword::LoadType::FORCE_WIND));
        m_target = form.targetSpin;
        m_direction = form.directionCombo;
        for (int value = static_cast<int>(EnumKeyword::Direction::X);
             value <= static_cast<int>(EnumKeyword::Direction::RZ); ++value)
        {
            const auto direction = static_cast<EnumKeyword::Direction>(value);
            m_direction->addItem(directionName(direction), value);
        }
        m_value = form.valueSpin;
        auto* buttons = form.buttonBox;
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        connect(buttons, &QDialogButtonBox::accepted, this, &LoadEditorDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(m_type, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { updateTargetState(); });
        if (load)
        {
            m_name->setText(load->m_Name);
            int stepIndex = m_step->findData(load->m_StepId);
            if (stepIndex < 0)
            {
                m_step->addItem(QStringLiteral("Step-%1（缺失）").arg(load->m_StepId), load->m_StepId);
                stepIndex = m_step->count() - 1;
            }
            m_step->setCurrentIndex(stepIndex);
            m_type->setCurrentIndex(qMax(0, m_type->findData(static_cast<int>(load->m_LoadType))));
            m_direction->setCurrentIndex(qMax(0,
                m_direction->findData(static_cast<int>(load->m_Direction))));
            const int targetId = loadTargetId(load);
            if (targetId > 0)
                m_target->setValue(targetId);
            m_value->setValue(loadValue(load));
        }
        else
        {
            if (m_step->count() > 1)
                m_step->setCurrentIndex(1);
            if (!structure->m_Nodes.empty())
                m_target->setValue(structure->m_Nodes.begin()->first);
        }
        updateTargetState();
    }

    std::shared_ptr<LoadBase> value() const
    {
        return m_result;
    }

protected:
    void accept() override
    {
        const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
        std::shared_ptr<LoadBase> result;
        if (type == EnumKeyword::LoadType::FORCE_NODE)
        {
            const auto node = m_structure->FindNode(m_target->value());
            if (!node)
            {
                QMessageBox::warning(this, QStringLiteral("无效目标"),
                    QStringLiteral("节点 %1 不存在。").arg(m_target->value()));
                return;
            }
            auto item = std::make_shared<Force_Node>();
            item->m_pNode = node;
            item->m_Value = m_value->value();
            result = item;
        }
        else if (type == EnumKeyword::LoadType::FORCE_ELEMENT)
        {
            const auto element = m_structure->FindElement(m_target->value());
            if (!element)
            {
                QMessageBox::warning(this, QStringLiteral("无效目标"),
                    QStringLiteral("单元 %1 不存在。").arg(m_target->value()));
                return;
            }
            auto item = std::make_shared<Force_Element>();
            item->m_pElement = element;
            item->m_Value = m_value->value();
            result = item;
        }
        else if (type == EnumKeyword::LoadType::FORCE_GRAVITY)
        {
            auto item = std::make_shared<Force_Gravity>();
            item->m_g = m_value->value();
            result = item;
        }
        else if (type == EnumKeyword::LoadType::FORCE_WIND)
        {
            auto item = std::make_shared<Force_Wind>();
            item->m_velocity = m_value->value();
            result = item;
        }
        if (!result)
            return;
        result->m_Id = m_id;
        result->m_Name = m_name->text().trimmed();
        result->m_StepId = m_step->currentData().toInt();
        result->m_Direction = static_cast<EnumKeyword::Direction>(m_direction->currentData().toInt());
        if (m_original)
        {
            result->m_StartTime = m_original->m_StartTime;
            result->m_EndTime = m_original->m_EndTime;
            result->m_FunctionType = m_original->m_FunctionType;
            result->m_Amplitude = m_original->m_Amplitude;
            result->m_Frequency = m_original->m_Frequency;
            result->m_Phase = m_original->m_Phase;
            result->m_Offset = m_original->m_Offset;
            result->m_RampT0 = m_original->m_RampT0;
            result->m_RampT1 = m_original->m_RampT1;
            result->m_Decay = m_original->m_Decay;
            result->m_Period = m_original->m_Period;
            result->m_DutyCycle = m_original->m_DutyCycle;
            const auto oldWind = std::dynamic_pointer_cast<Force_Wind>(m_original);
            const auto newWind = std::dynamic_pointer_cast<Force_Wind>(result);
            if (oldWind && newWind)
                newWind->m_windDensity = oldWind->m_windDensity;
        }
        m_result = result;
        QDialog::accept();
    }

private:
    void updateTargetState()
    {
        const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
        const bool hasTarget = type == EnumKeyword::LoadType::FORCE_NODE
            || type == EnumKeyword::LoadType::FORCE_ELEMENT;
        m_target->setEnabled(hasTarget);
        m_target->setToolTip(type == EnumKeyword::LoadType::FORCE_NODE
            ? QStringLiteral("节点编号") : type == EnumKeyword::LoadType::FORCE_ELEMENT
            ? QStringLiteral("单元编号") : QStringLiteral("该荷载作用于整个模型"));
    }

    int m_id = 0;
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<LoadBase> m_original;
    std::shared_ptr<LoadBase> m_result;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QComboBox* m_type = nullptr;
    QSpinBox* m_target = nullptr;
    QComboBox* m_direction = nullptr;
    QDoubleSpinBox* m_value = nullptr;
};

QString constraintDisplayName(int id, const std::shared_ptr<Constraint>& constraint)
{
    return constraint && !constraint->m_Name.trimmed().isEmpty()
        ? constraint->m_Name.trimmed() : QStringLiteral("Constraint-%1").arg(id);
}

class ConstraintEditorDialog final : public QDialog
{
public:
    ConstraintEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
        const std::shared_ptr<Constraint>& constraint, QWidget* parent)
        : QDialog(parent)
        , m_structure(structure)
    {
        Ui::ConstraintEditorDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("analysisConstraintEditorDialog"));
        setWindowTitle(constraint ? QStringLiteral("编辑约束") : QStringLiteral("新增约束"));
        setModal(true);
        m_name = form.nameEdit;
        m_name->setPlaceholderText(QStringLiteral("Constraint-%1").arg(id));
        m_step = form.stepCombo;
        m_step->addItem(QStringLiteral("初始 / 全局"), 0);
        for (const auto& [stepId, step] : structure->m_AnalysisStep)
            if (step)
                m_step->addItem(stepDisplayName(stepId, step), stepId);
        m_node = form.nodeSpin;
        m_direction = form.directionCombo;
        for (int value = static_cast<int>(EnumKeyword::Direction::X);
             value <= static_cast<int>(EnumKeyword::Direction::RZ); ++value)
        {
            const auto direction = static_cast<EnumKeyword::Direction>(value);
            m_direction->addItem(directionName(direction), value);
        }
        m_value = form.valueSpin;
        auto* buttons = form.buttonBox;
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        connect(buttons, &QDialogButtonBox::accepted, this, &ConstraintEditorDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        if (constraint)
        {
            m_name->setText(constraint->m_Name);
            m_step->setCurrentIndex(qMax(0, m_step->findData(constraint->m_StepId)));
            const auto node = constraint->m_pNode.lock();
            if (node)
                m_node->setValue(node->m_Id);
            m_direction->setCurrentIndex(qMax(0,
                m_direction->findData(static_cast<int>(constraint->m_Direction))));
            m_value->setValue(constraint->m_Value);
        }
        else
        {
            if (m_step->count() > 1)
                m_step->setCurrentIndex(1);
            if (!structure->m_Nodes.empty())
                m_node->setValue(structure->m_Nodes.begin()->first);
        }
    }

    QString name() const
    {
        return m_name->text().trimmed();
    }
    int stepId() const
    {
        return m_step->currentData().toInt();
    }
    std::shared_ptr<Node> node() const
    {
        return m_nodeValue;
    }
    EnumKeyword::Direction direction() const
    {
        return static_cast<EnumKeyword::Direction>(m_direction->currentData().toInt());
    }
    double displacement() const
    {
        return m_value->value();
    }

protected:
    void accept() override
    {
        m_nodeValue = m_structure->FindNode(m_node->value());
        if (!m_nodeValue)
        {
            QMessageBox::warning(this, QStringLiteral("无效节点"),
                QStringLiteral("节点 %1 不存在。").arg(m_node->value()));
            return;
        }
        QDialog::accept();
    }

private:
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<Node> m_nodeValue;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QSpinBox* m_node = nullptr;
    QComboBox* m_direction = nullptr;
    QDoubleSpinBox* m_value = nullptr;
};

QTableWidgetItem* tableItem(const QString& text, int id = -1)
{
    auto* item = new QTableWidgetItem(text);
    if (id >= 0)
        item->setData(Qt::UserRole, id);
    return item;
}
}

AnalysisManagerDialog::AnalysisManagerDialog(
    const std::shared_ptr<StructureData>& structure,
    Page initialPage,
    QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
    , m_page(initialPage)
{
    Ui::AnalysisManagerDialogClass form;
    form.setupUi(this);
    const bool isStepManager = initialPage == Page::Steps;
    const bool isLoadManager = initialPage == Page::Loads;
    setObjectName(isStepManager ? QStringLiteral("analysisStepManagerDialog")
        : isLoadManager ? QStringLiteral("analysisLoadManagerDialog")
        : QStringLiteral("analysisConstraintManagerDialog"));
    setWindowTitle(isStepManager ? QStringLiteral("分析步管理器")
        : isLoadManager ? QStringLiteral("荷载管理器")
        : QStringLiteral("边界条件管理器"));
    setWindowIcon(managerIcon(isStepManager ? ManagerGlyph::Steps
        : isLoadManager ? ManagerGlyph::Load : ManagerGlyph::Constraint, palette()));
    setModal(false);
    setWindowModality(Qt::NonModal);
    resize(920, 560);
    QTableWidget* table = form.resourceTable;
    applyCenteredTableAppearance(table);
    QPushButton* addButton = form.addButton;
    QPushButton* editButton = form.editButton;
    QPushButton* deleteButton = form.deleteButton;
    std::function<void()> addHandler;
    std::function<void()> editHandler;
    std::function<void()> deleteHandler;
    QStringList headers;
    if (isStepManager)
    {
        m_stepTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("类型"), QStringLiteral("总时间"),
            QStringLiteral("增量"), QStringLiteral("生效荷载数"), QStringLiteral("生效约束数") };
        addHandler = [this]() { editStep(); };
        editHandler = [this]() { editStep(currentId(m_stepTable)); };
        deleteHandler = [this]() { deleteSelectedStep(); };
    }
    else if (isLoadManager)
    {
        m_loadTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("分析步"), QStringLiteral("类型"),
            QStringLiteral("目标"), QStringLiteral("方向"), QStringLiteral("数值") };
        addHandler = [this]() { editLoad(); };
        editHandler = [this]() { editLoad(currentId(m_loadTable)); };
        deleteHandler = [this]() { deleteSelectedLoad(); };
    }
    else
    {
        m_constraintTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("分析步"), QStringLiteral("节点"),
            QStringLiteral("方向"), QStringLiteral("位移值") };
        addHandler = [this]() { editConstraint(); };
        editHandler = [this]() { editConstraint(currentId(m_constraintTable)); };
        deleteHandler = [this]() { deleteSelectedConstraint(); };
    }
    table->setColumnCount(headers.size());
    table->setHorizontalHeaderLabels(headers);
    table->verticalHeader()->setVisible(false);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    addButton->setObjectName(QStringLiteral("analysisAddButton"));
    editButton->setObjectName(QStringLiteral("analysisEditButton"));
    deleteButton->setObjectName(QStringLiteral("analysisDeleteButton"));
    addButton->setIcon(managerIcon(ManagerGlyph::Add, palette()));
    editButton->setIcon(managerIcon(ManagerGlyph::Edit, palette()));
    deleteButton->setIcon(managerIcon(ManagerGlyph::Delete, palette()));
    addButton->setIconSize(QSize(18, 18));
    editButton->setIconSize(QSize(18, 18));
    deleteButton->setIconSize(QSize(18, 18));
    for (QPushButton* button : { addButton, editButton, deleteButton })
        button->setMinimumWidth(qMax(92, button->sizeHint().width() + 4));
    connect(addButton, &QPushButton::clicked, this, addHandler);
    connect(editButton, &QPushButton::clicked, this, editHandler);
    connect(deleteButton, &QPushButton::clicked, this, deleteHandler);
    connect(table, &QTableWidget::itemDoubleClicked, this,
        [this, table, editHandler](QTableWidgetItem* item) {
            if (table == m_stepTable && item && item->column() == 4)
            {
                if (m_openManagerCallback)
                    m_openManagerCallback(Page::Loads);
                return;
            }
            if (table == m_stepTable && item && item->column() == 5)
            {
                if (m_openManagerCallback)
                    m_openManagerCallback(Page::Constraints);
                return;
            }
            editHandler();
        });
    connect(table, &QTableWidget::itemSelectionChanged, this,
        [table, editButton, deleteButton]() {
            const bool selected = table->currentRow() >= 0;
            editButton->setEnabled(selected);
            deleteButton->setEnabled(selected);
        });
    form.buttonBox->setObjectName(QStringLiteral("analysisManagerButtonBox"));
    form.buttonBox->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    connect(form.buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    refreshAll();
}

void AnalysisManagerDialog::setModelChangedCallback(
    std::function<void(const QSet<int>&)> callback)
{
    m_modelChangedCallback = std::move(callback);
}

void AnalysisManagerDialog::setOpenManagerCallback(std::function<void(Page)> callback)
{
    m_openManagerCallback = std::move(callback);
}

void AnalysisManagerDialog::refreshFromModel()
{
    refreshAll();
}

QSet<int> AnalysisManagerDialog::affectedStepsFrom(int firstStepId) const
{
    QSet<int> affected;
    if (!m_structure)
        return affected;
    for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
    {
        if (step && (firstStepId <= 0 || stepId >= firstStepId))
            affected.insert(stepId);
    }
    return affected;
}

void AnalysisManagerDialog::markModelChanged(const QSet<int>& affectedStepIds)
{
    m_modelChanged = true;
    refreshAll();
    if (m_modelChangedCallback)
        m_modelChangedCallback(affectedStepIds);
}

void AnalysisManagerDialog::refreshAll()
{
    refreshStepTable();
    refreshLoadTable();
    refreshConstraintTable();
}

int AnalysisManagerDialog::currentId(QTableWidget* table) const
{
    if (!table || table->currentRow() < 0 || !table->item(table->currentRow(), 0))
        return -1;
    return table->item(table->currentRow(), 0)->data(Qt::UserRole).toInt();
}

void AnalysisManagerDialog::selectId(QTableWidget* table, int id) const
{
    if (!table)
        return;
    for (int row = 0; row < table->rowCount(); ++row)
    {
        if (table->item(row, 0) && table->item(row, 0)->data(Qt::UserRole).toInt() == id)
        {
            table->selectRow(row);
            table->scrollToItem(table->item(row, 0));
            return;
        }
    }
    table->clearSelection();
}

void AnalysisManagerDialog::refreshStepTable(int preferredId)
{
    if (!m_stepTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_stepTable);
    m_stepTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
    {
        if (!step)
            continue;
        int loadCount = 0;
        int constraintCount = 0;
        for (const auto& [loadId, load] : m_structure->m_Load)
        {
            Q_UNUSED(loadId);
            if (isEffectiveAtStep(load, stepId))
                ++loadCount;
        }
        for (const auto& [constraintId, constraint] : m_structure->m_Constraint)
        {
            Q_UNUSED(constraintId);
            if (isEffectiveAtStep(constraint, stepId))
                ++constraintCount;
        }
        const int row = m_stepTable->rowCount();
        m_stepTable->insertRow(row);
        m_stepTable->setItem(row, 0, tableItem(stepDisplayName(stepId, step), stepId));
        m_stepTable->setItem(row, 1, tableItem(step->m_Type == EnumKeyword::StepType::STATIC
            ? QStringLiteral("静力") : step->m_Type == EnumKeyword::StepType::DYNAMIC
            ? QStringLiteral("动力") : QStringLiteral("未知")));
        m_stepTable->setItem(row, 2, tableItem(QString::number(step->m_Time, 'g', 10)));
        m_stepTable->setItem(row, 3, tableItem(QString::number(step->m_StepSize, 'g', 10)));
        m_stepTable->setItem(row, 4, tableItem(QString::number(loadCount)));
        m_stepTable->setItem(row, 5, tableItem(QString::number(constraintCount)));
    }
    selectId(m_stepTable, preferredId);
}
void AnalysisManagerDialog::refreshLoadTable(int preferredId)
{
    if (!m_loadTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_loadTable);
    m_loadTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [loadId, load] : m_structure->m_Load)
    {
        if (!load)
            continue;
        QString stepName = QStringLiteral("初始 / 全局");
        if (load->m_StepId > 0)
        {
            const auto step = m_structure->m_AnalysisStep.find(load->m_StepId);
            stepName = step != m_structure->m_AnalysisStep.end()
                ? stepDisplayName(step->first, step->second)
                : QStringLiteral("Step-%1（缺失）").arg(load->m_StepId);
        }
        const int targetId = loadTargetId(load);
        const QString target = targetId > 0
            ? (load->m_LoadType == EnumKeyword::LoadType::FORCE_NODE
                ? QStringLiteral("节点 %1").arg(targetId) : QStringLiteral("单元 %1").arg(targetId))
            : QStringLiteral("全模型");
        const int row = m_loadTable->rowCount();
        m_loadTable->insertRow(row);
        m_loadTable->setItem(row, 0, tableItem(loadDisplayName(loadId, load), loadId));
        m_loadTable->setItem(row, 1, tableItem(stepName));
        m_loadTable->setItem(row, 2, tableItem(loadTypeName(load->m_LoadType)));
        m_loadTable->setItem(row, 3, tableItem(target));
        m_loadTable->setItem(row, 4, tableItem(directionName(load->m_Direction)));
        m_loadTable->setItem(row, 5, tableItem(QString::number(loadValue(load), 'g', 12)));
    }
    selectId(m_loadTable, preferredId);
}
void AnalysisManagerDialog::refreshConstraintTable(int preferredId)
{
    if (!m_constraintTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_constraintTable);
    m_constraintTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [constraintId, constraint] : m_structure->m_Constraint)
    {
        if (!constraint)
            continue;
        QString stepName = QStringLiteral("初始 / 全局");
        if (constraint->m_StepId > 0)
        {
            const auto step = m_structure->m_AnalysisStep.find(constraint->m_StepId);
            stepName = step != m_structure->m_AnalysisStep.end()
                ? stepDisplayName(step->first, step->second)
                : QStringLiteral("Step-%1（缺失）").arg(constraint->m_StepId);
        }
        const auto node = constraint->m_pNode.lock();
        const int row = m_constraintTable->rowCount();
        m_constraintTable->insertRow(row);
        m_constraintTable->setItem(row, 0,
            tableItem(constraintDisplayName(constraintId, constraint), constraintId));
        m_constraintTable->setItem(row, 1, tableItem(stepName));
        m_constraintTable->setItem(row, 2,
            tableItem(node ? QString::number(node->m_Id) : QStringLiteral("--")));
        m_constraintTable->setItem(row, 3, tableItem(directionName(constraint->m_Direction)));
        m_constraintTable->setItem(row, 4,
            tableItem(QString::number(constraint->m_Value, 'g', 12)));
    }
    selectId(m_constraintTable, preferredId);
}
void AnalysisManagerDialog::editStep(int stepId)
{
    if (!m_structure)
        return;
    std::shared_ptr<AnalysisStep> existing;
    if (stepId >= 0)
    {
        const auto found = m_structure->m_AnalysisStep.find(stepId);
        if (found == m_structure->m_AnalysisStep.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        stepId = nextId(m_structure->m_AnalysisStep);
    }

    StepEditorDialog editor(stepId, existing, this);
    if (editor.exec() != QDialog::Accepted)
        return;
    const AnalysisStepConfig value = editor.config();
    if (existing)
    {
        existing->m_Name = value.name.isEmpty() ? QStringLiteral("Step-%1").arg(stepId) : value.name;
        existing->m_Type = value.type;
        existing->isDynamic = value.type == EnumKeyword::StepType::DYNAMIC;
        existing->m_Time = value.totalTime;
        existing->m_StepSize = value.stepSize;
        existing->m_Tolerance = value.tolerance;
        existing->m_MaxIterations = value.maxIterations;
    }
    else
    {
        m_structure->AddAnalysisStep(value);
    }
    markModelChanged({ stepId });
    selectId(m_stepTable, stepId);
}
void AnalysisManagerDialog::editLoad(int loadId)
{
    if (!m_structure)
        return;
    if (loadId < 0 && m_structure->m_AnalysisStep.empty())
    {
        QMessageBox::information(this, QStringLiteral("无法新增荷载"),
            QStringLiteral("请先创建至少一个分析步。"));
        return;
    }
    std::shared_ptr<LoadBase> existing;
    if (loadId >= 0)
    {
        const auto found = m_structure->m_Load.find(loadId);
        if (found == m_structure->m_Load.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        loadId = nextId(m_structure->m_Load);
    }
    const int oldStepId = existing ? existing->m_StepId : -1;
    LoadEditorDialog editor(loadId, m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted || !editor.value())
        return;
    m_structure->m_Load[loadId] = editor.value();
    QSet<int> affected = affectedStepsFrom(editor.value()->m_StepId);
    if (oldStepId >= 0)
        affected.unite(affectedStepsFrom(oldStepId));
    markModelChanged(affected);
    selectId(m_loadTable, loadId);
}
void AnalysisManagerDialog::editConstraint(int constraintId)
{
    if (!m_structure)
        return;
    std::shared_ptr<Constraint> existing;
    if (constraintId >= 0)
    {
        const auto found = m_structure->m_Constraint.find(constraintId);
        if (found == m_structure->m_Constraint.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        constraintId = nextId(m_structure->m_Constraint);
    }
    const int oldStepId = existing ? existing->m_StepId : -1;
    ConstraintEditorDialog editor(constraintId, m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted || !editor.node())
        return;
    auto constraint = existing ? existing : std::make_shared<Constraint>();
    constraint->m_Id = constraintId;
    constraint->m_Name = editor.name();
    constraint->m_StepId = editor.stepId();
    constraint->m_pNode = editor.node();
    constraint->m_Direction = editor.direction();
    constraint->m_Value = editor.displacement();
    m_structure->m_Constraint[constraintId] = constraint;
    QSet<int> affected = affectedStepsFrom(constraint->m_StepId);
    if (oldStepId >= 0)
        affected.unite(affectedStepsFrom(oldStepId));
    markModelChanged(affected);
    selectId(m_constraintTable, constraintId);
}
void AnalysisManagerDialog::deleteSelectedStep()
{
    if (!m_structure)
        return;
    const int stepId = currentId(m_stepTable);
    const auto found = m_structure->m_AnalysisStep.find(stepId);
    if (found == m_structure->m_AnalysisStep.end())
        return;
    int loadCount = 0;
    int constraintCount = 0;
    for (const auto& [id, load] : m_structure->m_Load)
    {
        Q_UNUSED(id);
        if (load && load->m_StepId == stepId)
            ++loadCount;
    }
    for (const auto& [id, constraint] : m_structure->m_Constraint)
    {
        Q_UNUSED(id);
        if (constraint && constraint->m_StepId == stepId)
            ++constraintCount;
    }
    const QString prompt = QStringLiteral("确定删除“%1”吗？\n同时会删除归属于该步的 %2 个荷载和 %3 个约束。")
        .arg(stepDisplayName(stepId, found->second)).arg(loadCount).arg(constraintCount);
    if (QMessageBox::question(this, QStringLiteral("删除分析步"), prompt,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;

    // Loads and constraints introduced by this step are inherited by later
    // steps.  Capture that dependency range before erasing the step so only
    // the genuinely affected later cases are invalidated.
    const QSet<int> affectedStepIds = affectedStepsFrom(stepId);
    m_structure->m_AnalysisStep.erase(found);
    for (auto it = m_structure->m_Load.begin(); it != m_structure->m_Load.end(); )
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_Load.erase(it) : std::next(it);
    for (auto it = m_structure->m_Constraint.begin(); it != m_structure->m_Constraint.end(); )
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_Constraint.erase(it) : std::next(it);
    markModelChanged(affectedStepIds);
}
void AnalysisManagerDialog::deleteSelectedLoad()
{
    if (!m_structure)
        return;
    const int loadId = currentId(m_loadTable);
    const auto found = m_structure->m_Load.find(loadId);
    if (found == m_structure->m_Load.end())
        return;
    if (QMessageBox::question(this, QStringLiteral("删除荷载"),
        QStringLiteral("确定删除“%1”吗？").arg(loadDisplayName(loadId, found->second)),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;
    const int firstAffectedStep = found->second ? found->second->m_StepId : 0;
    m_structure->m_Load.erase(found);
    markModelChanged(affectedStepsFrom(firstAffectedStep));
}
void AnalysisManagerDialog::deleteSelectedConstraint()
{
    if (!m_structure)
        return;
    const int constraintId = currentId(m_constraintTable);
    const auto found = m_structure->m_Constraint.find(constraintId);
    if (found == m_structure->m_Constraint.end())
        return;
    if (QMessageBox::question(this, QStringLiteral("删除约束"),
        QStringLiteral("确定删除“%1”吗？").arg(constraintDisplayName(constraintId, found->second)),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;
    const int firstAffectedStep = found->second ? found->second->m_StepId : 0;
    m_structure->m_Constraint.erase(found);
    markModelChanged(affectedStepsFrom(firstAffectedStep));
}
