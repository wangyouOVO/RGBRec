#ifndef RECON_H
#define RECON_H

#include <QMainWindow>

namespace Ui {
class Recon;
}

class Recon : public QMainWindow
{
    Q_OBJECT

public:
    explicit Recon(QWidget *parent = nullptr);
    ~Recon();

private:
    Ui::Recon *ui;
};

#endif // RECON_H
