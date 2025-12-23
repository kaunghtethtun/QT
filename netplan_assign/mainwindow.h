#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:
    bool runNetplanGenerateOnRobot();
    bool runNetplanTryOnRobot();
    void on_pushButton_generate_clicked();
    void on_pushButton_generate_netplan_clicked();
    void on_pushButton_try_netplan_clicked();

private:
    void showNotification(const QString& message, bool success);

private:
    Ui::MainWindow *ui;
    // Store robot connection info
    QString robotIp;
    QString robotUsername;
    QString robotPassword;
    bool robotInfoSet = false;
    bool requestRobotInfo();
};
#endif // MAINWINDOW_H
