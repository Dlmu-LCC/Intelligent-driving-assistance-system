#include "source.h"
#include "ui_source.h"

Source::Source(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Source)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose,true);

}

Source::~Source()
{
    delete ui;
}

void Source::on_CameraBtn_released(){
    emit SendModeAndData(false,false);
    this->close();
}


void Source::on_SimulationBtn_released(){
    emit SendModeAndData(true,true);
    this->close();
}


void Source::on_SimWithoutDataBtn_released()
{
    emit SendModeAndData(true,false);
    this->close();
}

