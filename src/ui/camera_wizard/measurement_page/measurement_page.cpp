#include "measurement_page.h"

MeasurementPage::MeasurementPage(QWidget *parent) :
    QWizardPage(parent),
    ui(new Ui::MeasurementPage)
{
    ui->setupUi(this);
 
    // 字段
    registerField("车宽度", ui->carWidthInput, "value", SIGNAL(valueChanged(double)));
    registerField("地毯宽度", ui->carpetWidthInput, "value", SIGNAL(valueChanged(double)));
    registerField("车到地毯的距离", ui->carToCarpetDistanceInput, "value", SIGNAL(valueChanged(double)));
    registerField("地毯长度", ui->carpetLengthInput, "value", SIGNAL(valueChanged(double)));

}
