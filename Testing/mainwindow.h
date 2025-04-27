#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcpp/rclcpp.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onGoToListClicked();
    void onPublishClicked();
    void onSubscribeClicked();

private:
    Ui::MainWindow *ui;
    std::shared_ptr<rclcpp::Node> ros2_node_;
};

#endif // MAINWINDOW_H
