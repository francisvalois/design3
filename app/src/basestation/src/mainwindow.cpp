#include "mainwindow.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow), baseStation(argc, argv) {
    ui->setupUi(this);

    QObject::connect(&baseStation, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_StartSequenceButton_clicked() {
    baseStation.init();
}
