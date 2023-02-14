#ifndef RECON_H
#define RECON_H

#include <QMainWindow>
#include <QImage>
#include <QFileInfo>
#include <QFileDialog>
#include <QDebug>
#include "fileio.h"
#include "utils.h"
namespace Ui {
class Recon;
}

class Recon : public QMainWindow
{
    Q_OBJECT

public:
    explicit Recon(QWidget *parent = nullptr);
    ~Recon();
    ProjectInfo* projectInfo;
    FileIO* fileIO = new FileIO();
    void initWin();
    void openFile();
    void saveFile();
    void updateState();
private:
    Ui::Recon *ui;
};

#endif // RECON_H
