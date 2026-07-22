#pragma once

#include <QFileDialog>
#include <QItemSelection>

class QAbstractItemView;
class QRubberBand;

class ModelImportFileDialog final : public QFileDialog
{
public:
    explicit ModelImportFileDialog(const QString& directory, QWidget* parent = nullptr);
    ModelImportFileDialog(const QString& directory, const QString& title, const QString& nameFilter,
                          QFileDialog::FileMode fileMode, QWidget* parent = nullptr);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void configureInternalWidgets();
    void configureRubberBandSelection(QAbstractItemView* view);
    void updateRubberBandSelection(const QRect& selectionRect);

    QAbstractItemView* m_rubberBandView = nullptr;
    QRubberBand* m_rubberBand = nullptr;
    QPoint m_rubberBandOrigin;
    QItemSelection m_selectionBeforeDrag;
    Qt::KeyboardModifiers m_rubberBandModifiers = Qt::NoModifier;
    bool m_rubberBandPressed = false;
    bool m_rubberBandDragging = false;
};
