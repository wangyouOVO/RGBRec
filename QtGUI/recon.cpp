#include "recon.h"
#include "ui_recon.h"
#include "QDialog"

Recon::Recon(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Recon)
{

    ui->setupUi(this);
    initWin();
    QObject::connect(ui->actionimage,&QAction::triggered,[=](){
        ui->stackedWidget->setCurrentIndex(0);
    });
    QObject::connect(ui->action3Dmodel,&QAction::triggered,[=](){
        ui->stackedWidget->setCurrentIndex(1);
    });
    QObject::connect(ui->actionopen,&QAction::triggered,[&](){
        openFile();
        ui->logListWidget->addItem("file opened successfully");
    });
    QObject::connect(ui->actionsave,&QAction::triggered,[&](){
        saveFile();
        ui->logListWidget->addItem("file saved successfully");
    });
    QObject::connect(ui->imagePathListWidget, &QListWidget::itemClicked,
                     this, [&](QListWidgetItem *item){
        Recon::onListImageItemClicked(item);
    });
    QObject::connect(ui->actionchoose_photo,&QAction::triggered,[&](){
        //add photo
        addPhoto(this);
        ui->logListWidget->addItem("add photo successfully");
    });
    QObject::connect(ui->actionclose,&QAction::triggered,[&](){
        closeFile();
        ui->logListWidget->addItem("close project successfully");
    });
}

Recon::~Recon()
{
    delete ui;
}

void Recon::initWin(){
    ui->stateTableWidget->setColumnCount(2);
    ui->stateTableWidget->setHorizontalHeaderLabels(QStringList()<<"property"<<"value");
    ui->stateTableWidget->setRowCount(6);
    ui->stateTableWidget->verticalHeader()->setHidden(true);
    ui->logListWidget->addItem("System initialization");

    QStringList propertyList;
    propertyList<<"name"<<"imageNum"<<"isCalibra"<<"isSfm"<<"isDenseRec"<<"isSurface";
    for(int i = 0;i<6;i++){
        ui->stateTableWidget->setItem(i,0,new QTableWidgetItem(propertyList[i]));
    }
}

void Recon::openFile(){
    projectInfo = fileIO->getInfoFromFile(this);
    updateState();
}

void Recon::saveFile(){
    fileIO->printInfoTofile(projectInfo);
}

void Recon::updateState(){
    QStringList valueList;
    valueList << str2qstr(projectInfo->projectName) << QString::number(projectInfo->imagesNum) << showBool(projectInfo->isCalitration) <<\
                 showBool(projectInfo->isSfm) << showBool(projectInfo->isDenseRec) <<showBool(projectInfo->isSurface);
    for(int i = 0;i<6;i++){
        ui->stateTableWidget->setItem(i,1,new QTableWidgetItem(valueList[i]));
    }
    ui->imagePathListWidget->clear();
    for(auto item : projectInfo->imagePaths){
        ui->imagePathListWidget->addItem(str2qstr(getImageNameFromPath(item)));
    }
}

void Recon::onListImageItemClicked(QListWidgetItem* item){
    qDebug()<<item->text();
    string imagePath = getPathFromFullPath(projectInfo->imagePaths[0]) + qstr2str(item->text());
//    debugstring("haha",imagePath);
    QImage img;
    if(! ( img.load(str2qstr( imagePath)) ) ) //加载图像
    {
        QMessageBox::information(this,
                                 tr("打开图像失败"),
                                 tr("打开图像失败!"));
        return;
    }
    ui->imagelabel->setPixmap(QPixmap::fromImage(img).scaled(ui->imagelabel->size()));
    // QDialog *dlg = new QDialog (this);
    // dlg->resize(200,200);
    // dlg->setAttribute(Qt::WA_DeleteOnClose);
    // QLabel *haha = new QLabel (dlg);
    // haha->resize(200,200);
    // haha->setPixmap(QPixmap::fromImage(img).scaled(haha->size()));
    // dlg->show();

}

void Recon::addPhoto(QWidget* Rec){
    QStringList  OpenFile;
    OpenFile = QFileDialog::getOpenFileNames(Rec,
                                            "please choose an image file",
                                            "",
                                            "Image Files(*.JPG *.jpg *.PNG *.png);;All(*.*)");
    for(auto item : OpenFile){
        projectInfo->addImagePath(qstr2str(item));
    }
    updateState();
}

void Recon::closeFile(){
    saveFile();
    ProjectInfo* p;
    p = projectInfo;
    projectInfo->clearAllInfo();
    delete p;
    updateState();
}
