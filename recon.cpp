#include "recon.h"
#include "ui_recon.h"


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
    });
    QObject::connect(ui->actionsave,&QAction::triggered,[&](){
        saveFile();
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
    for(auto item : projectInfo->imagePaths){
        ui->imagePathListWidget->addItem(str2qstr(getImageNameFromPath(item)));
    }
}

