#ifndef SOURCE_H
#define SOURCE_H

#include <QWidget>
#include "ui_source.h"

namespace Ui {
class Source;
}

class Source : public QWidget
{
    Q_OBJECT

public:
    bool getMode();
    explicit Source(QWidget *parent = 0);
    ~Source();

private slots:
    void on_CameraBtn_released();

    void on_SimulationBtn_released();

    void on_SimWithoutDataBtn_released();

signals:
    void SendModeAndData(bool, bool);

private:
    Ui::Source *ui;
};

#endif // SOURCE_H
