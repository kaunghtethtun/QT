#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTreeWidgetItem>
#include <rclcpp/rclcpp.hpp> // ROS2
#include <memory>

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
    void onTopicButtonClicked();
    void onServiceButtonClicked();
    void onActionButtonClicked();

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;   // ROS2 node
};

#endif // MAINWINDOW_H
