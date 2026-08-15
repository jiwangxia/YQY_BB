#include "GUI/Dialogs/ModelImportFileDialog.h"

#include <QAbstractItemView>
#include <QApplication>
#include <QFileInfo>
#include <QFileSystemModel>
#include <QMouseEvent>
#include <QRubberBand>

ModelImportFileDialog::ModelImportFileDialog(const QString& directory, QWidget* parent)
    : ModelImportFileDialog(
          directory, QStringLiteral("导入模型"),
          QStringLiteral("模型文件 (*.bdf);;H5模型文件 (*.h5 *.hdf5);;杆塔模型文件 (*.txt);;ABAQUS文件 (*.inp)"),
          QFileDialog::ExistingFiles, parent)
{
}

ModelImportFileDialog::ModelImportFileDialog(const QString& directory, const QString& title, const QString& nameFilter,
                                             QFileDialog::FileMode fileMode, QWidget* parent)
    : QFileDialog(parent, title, directory, nameFilter)
{
    setObjectName(QStringLiteral("modelImportFileDialog"));
    setOption(QFileDialog::DontUseNativeDialog, true);
    setFileMode(fileMode);
    setAcceptMode(QFileDialog::AcceptOpen);
    setLabelText(QFileDialog::Accept, QStringLiteral("打开"));

    const QScreen* screen = QGuiApplication::screenAt(parent ? parent->frameGeometry().center() : QCursor::pos());
    if (!screen)
        screen = QGuiApplication::primaryScreen();
    const QSize available = screen ? screen->availableGeometry().size() : QSize(1280, 800);
    const int dialogWidth = qBound(760, qRound(available.width() * 0.58), 900);
    const int dialogHeight = qBound(520, qRound(available.height() * 0.64), 640);
    resize(dialogWidth, dialogHeight);
    setMinimumSize(qMin(720, dialogWidth), qMin(500, dialogHeight));

    setStyleSheet(QStringLiteral(R"QSS(
QFileDialog QToolButton {
    padding: 0;
    min-width: 34px;
    max-width: 34px;
    min-height: 30px;
    max-height: 30px;
    icon-size: 20px;
}
QFileDialog QListView#sidebar {
    min-width: 210px;
}
)QSS"));

    configureInternalWidgets();
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           configureInternalWidgets();
                       });
}

void ModelImportFileDialog::configureInternalWidgets()
{
    const QSize toolbarIconSize(20, 20);
    for (QToolButton* button : findChildren<QToolButton*>())
    {
        button->setIconSize(toolbarIconSize);
        button->setFixedSize(34, 30);
        button->setCursor(Qt::PointingHandCursor);
    }

    QListView* sidebar = findChild<QListView*>(QStringLiteral("sidebar"));
    for (QAbstractItemView* view : findChildren<QAbstractItemView*>())
    {
        view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        view->setTextElideMode(Qt::ElideMiddle);
    }

    if (sidebar)
    {
        sidebar->setMinimumWidth(210);
        sidebar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    }

    if (QTreeView* fileTree = findChild<QTreeView*>(QStringLiteral("treeView")))
    {
        if (fileMode() == QFileDialog::ExistingFiles)
            configureRubberBandSelection(fileTree);
        if (QHeaderView* header = fileTree->header())
        {
            header->setStretchLastSection(false);
            if (header->count() > 0)
                header->setSectionResizeMode(0, QHeaderView::Stretch);
            for (int column = 1; column < header->count(); ++column)
                header->setSectionResizeMode(column, QHeaderView::ResizeToContents);
        }
    }
    if (fileMode() == QFileDialog::ExistingFiles)
    {
        if (QListView* fileList = findChild<QListView*>(QStringLiteral("listView")))
            configureRubberBandSelection(fileList);
    }

    if (!sidebar)
        return;
    for (QSplitter* splitter : findChildren<QSplitter*>())
    {
        QWidget* sidebarPane = sidebar;
        while (sidebarPane && sidebarPane->parentWidget() != splitter)
            sidebarPane = sidebarPane->parentWidget();
        const int sidebarIndex = sidebarPane ? splitter->indexOf(sidebarPane) : -1;
        if (sidebarIndex < 0)
            continue;

        splitter->setChildrenCollapsible(false);
        splitter->setCollapsible(sidebarIndex, false);
        splitter->setStretchFactor(sidebarIndex, 0);
        if (splitter->count() > 1)
        {
            const int contentIndex = sidebarIndex == 0 ? 1 : 0;
            splitter->setStretchFactor(contentIndex, 1);
            QList<int> sizes = splitter->sizes();
            if (sizes.size() == splitter->count())
            {
                const int totalWidth = qMax(width() - 40, 720);
                sizes[sidebarIndex] = 220;
                sizes[contentIndex] = qMax(500, totalWidth - 220);
                splitter->setSizes(sizes);
            }
        }
        break;
    }
}

void ModelImportFileDialog::configureRubberBandSelection(QAbstractItemView* view)
{
    if (!view || view->property("yqyRubberBandSelection").toBool())
        return;

    view->setSelectionMode(QAbstractItemView::ExtendedSelection);
    view->setDragEnabled(false);
    view->viewport()->installEventFilter(this);
    view->setProperty("yqyRubberBandSelection", true);
}

bool ModelImportFileDialog::eventFilter(QObject* watched, QEvent* event)
{
    QAbstractItemView* view = nullptr;
    for (QAbstractItemView* candidate : findChildren<QAbstractItemView*>())
    {
        if (candidate->viewport() == watched)
        {
            view = candidate;
            break;
        }
    }
    if (!view || !view->property("yqyRubberBandSelection").toBool())
        return QFileDialog::eventFilter(watched, event);

    if (event->type() == QEvent::MouseButtonPress)
    {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() != Qt::LeftButton)
            return QFileDialog::eventFilter(watched, event);

        m_rubberBandView = view;
        m_rubberBandOrigin = mouseEvent->pos();
        m_selectionBeforeDrag = view->selectionModel()->selection();
        m_rubberBandModifiers = mouseEvent->modifiers();
        m_rubberBandPressed = true;
        m_rubberBandDragging = false;
        return QFileDialog::eventFilter(watched, event);
    }

    if (event->type() == QEvent::MouseMove && m_rubberBandPressed && m_rubberBandView == view)
    {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        if (!(mouseEvent->buttons() & Qt::LeftButton))
            return QFileDialog::eventFilter(watched, event);

        if (!m_rubberBandDragging &&
            (mouseEvent->pos() - m_rubberBandOrigin).manhattanLength() < QApplication::startDragDistance())
        {
            return QFileDialog::eventFilter(watched, event);
        }

        if (!m_rubberBand || m_rubberBand->parentWidget() != view->viewport())
        {
            delete m_rubberBand;
            m_rubberBand = new QRubberBand(QRubberBand::Rectangle, view->viewport());
        }
        m_rubberBandDragging = true;
        const QRect selectionRect = QRect(m_rubberBandOrigin, mouseEvent->pos()).normalized();
        m_rubberBand->setGeometry(selectionRect);
        m_rubberBand->show();
        updateRubberBandSelection(selectionRect);
        return true;
    }

    if (event->type() == QEvent::MouseButtonRelease && m_rubberBandPressed && m_rubberBandView == view)
    {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() != Qt::LeftButton)
            return QFileDialog::eventFilter(watched, event);

        const bool wasDragging = m_rubberBandDragging;
        if (m_rubberBand)
            m_rubberBand->hide();
        m_rubberBandPressed = false;
        m_rubberBandDragging = false;
        m_rubberBandView = nullptr;
        return wasDragging ? true : QFileDialog::eventFilter(watched, event);
    }

    return QFileDialog::eventFilter(watched, event);
}

void ModelImportFileDialog::updateRubberBandSelection(const QRect& selectionRect)
{
    if (!m_rubberBandView || !m_rubberBandView->selectionModel())
        return;

    QItemSelectionModel* selectionModel = m_rubberBandView->selectionModel();
    selectionModel->clearSelection();
    if (m_rubberBandModifiers & Qt::ControlModifier)
        selectionModel->select(m_selectionBeforeDrag, QItemSelectionModel::Select | QItemSelectionModel::Rows);

    const QModelIndex root = m_rubberBandView->rootIndex();
    const int rowCount = m_rubberBandView->model()->rowCount(root);
    for (int row = 0; row < rowCount; ++row)
    {
        const QModelIndex index = m_rubberBandView->model()->index(row, 0, root);
        if (!index.isValid() || !m_rubberBandView->visualRect(index).intersects(selectionRect))
            continue;
        const QString filePath = index.data(QFileSystemModel::FilePathRole).toString();
        if (!QFileInfo(filePath).isFile())
            continue;

        const bool wasSelected = m_selectionBeforeDrag.contains(index);
        const QItemSelectionModel::SelectionFlags flags =
            (m_rubberBandModifiers & Qt::ControlModifier) && wasSelected
                ? QItemSelectionModel::Deselect | QItemSelectionModel::Rows
                : QItemSelectionModel::Select | QItemSelectionModel::Rows;
        selectionModel->select(index, flags);
    }
}
