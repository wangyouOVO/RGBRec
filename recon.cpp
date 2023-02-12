#include "recon.h"
#include "ui_recon.h"

Recon::Recon(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Recon)
{
    ui->setupUi(this);
    QObject::connect(ui->actionimage,&QAction::triggered,[=](){
        ui->stackedWidget->setCurrentIndex(0);
    });
    QObject::connect(ui->action3Dmodel,&QAction::triggered,[=](){
        ui->stackedWidget->setCurrentIndex(1);
    });

}

Recon::~Recon()
{
    delete ui;
}
