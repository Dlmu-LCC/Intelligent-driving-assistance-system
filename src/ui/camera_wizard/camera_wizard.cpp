#include "camera_wizard.h"

using namespace std;

QWizardPage *createIntroPage()
{
    QWizardPage *page = new QWizardPage;
    page->setTitle("警告!!!");



    QLabel *label = new QLabel("这个向导将帮助你校准你的摄像头。\n"
                               "相机校准必须在有足够的仪器的合格车库中进行。\n"
                               "如果你知道你现在在做什么，那么你可以点击继续。\n");
    QFont ft;
    ft.setPointSize(20);
    label->setFont(ft);

    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}


QWizardPage *createFinishPage()
{
    QWizardPage *page = new QWizardPage;
    page->setTitle("完成");

    QLabel *label = new QLabel("相机校准完成。");
    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}


CameraWizard::CameraWizard(std::shared_ptr<CarStatus> car_status) {
    measurement_page = std::make_shared<MeasurementPage>(nullptr);
    this->car_status = car_status;
    this->addPage(createIntroPage());
    this->addPage(new InstructionPage(nullptr));
    this->addPage(measurement_page.get());
    this->addPage(new FourPointSelectPage(nullptr, car_status));
    this->addPage(createFinishPage());
    this->setWindowTitle("相机校准向导");
    connect(this->button(QWizard::FinishButton), SIGNAL(clicked()), this, SLOT(onFinishButtonClicked()));
}


void CameraWizard::onFinishButtonClicked() {

    float car_width = field("车宽度").toFloat();
    float carpet_width = field("地毯宽度").toFloat();
    float car_to_carpet_distance = field("车到地毯的距离").toFloat();
    float carpet_length = field("地毯长度").toFloat();
    float tl_x = field("tl_x").toFloat();
    float tl_y = field("tl_y").toFloat();
    float tr_x = field("tr_x").toFloat();
    float tr_y = field("tr_y").toFloat();
    float br_x = field("br_x").toFloat();
    float br_y = field("br_y").toFloat();
    float bl_x = field("bl_x").toFloat();
    float bl_y = field("bl_y").toFloat();

    // 向文件中写入新数据
    if( remove( SMARTCAM_CAMERA_CALIB_FILE ) != 0 )
        cout << "错误！无法删除数据！" << endl;
    std::ofstream calib_file;
    calib_file.open (SMARTCAM_CAMERA_CALIB_FILE);
    calib_file << "车宽度 " << car_width << endl;
    calib_file << "地毯宽度 " << carpet_width << endl;
    calib_file << "车到地毯的距离 " << car_to_carpet_distance << endl;
    calib_file << "地毯长度 " << carpet_length << endl;
    calib_file << "tl_x " << tl_x << endl;
    calib_file << "tl_y " << tl_y << endl;
    calib_file << "tr_x " << tr_x << endl;
    calib_file << "tr_y " << tr_y << endl;
    calib_file << "br_x " << br_x << endl;
    calib_file << "br_y " << br_y << endl;
    calib_file << "bl_x " << bl_x << endl;
    calib_file << "bl_y " << bl_y << endl;
    calib_file.close();

    emit updateCameraModel(
        car_width, carpet_width, car_to_carpet_distance, carpet_length,
        tl_x, tl_y, tr_x, tr_y, br_x, br_y, bl_x, bl_y
    );
   
}
