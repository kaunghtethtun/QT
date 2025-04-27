#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    // Force start from Page 1
    ui->stackedWidget->setCurrentIndex(0);

    rclcpp::init(0, nullptr);
    ros2_node_ = rclcpp::Node::make_shared("qt_topic_list_node");

    connect(ui->pushButton_GoToList, &QPushButton::clicked, this, &MainWindow::onGoToListClicked);
    connect(ui->pushButton_Publish, &QPushButton::clicked, this, &MainWindow::onPublishClicked);
    connect(ui->pushButton_Subscribe, &QPushButton::clicked, this, &MainWindow::onSubscribeClicked);
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::onGoToListClicked()
{
    if (ros2_node_) {
        ui->listWidget_Topics->clear();
        auto topic_names_and_types = ros2_node_->get_topic_names_and_types();

        for (auto &pair : topic_names_and_types) {
            QString topicName = QString::fromStdString(pair.first);
            ui->listWidget_Topics->addItem(topicName);
        }
    }

    // Switch to second page
    ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::onPublishClicked()
{
    QString selectedTopic = ui->listWidget_Topics->currentItem()->text();
    qDebug() << "Publish clicked for topic:" << selectedTopic;
    // Later: you can write code to publish here
}

void MainWindow::onSubscribeClicked()
{
    QString selectedTopic = ui->listWidget_Topics->currentItem()->text();
    qDebug() << "Subscribe clicked for topic:" << selectedTopic;
    // Later: you can write code to subscribe here
}
