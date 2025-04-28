#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/graph_listener.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Initialize ROS2 node (anonymous node)
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("qt_gui_node");

    // Connect buttons
    connect(ui->topicButton, &QPushButton::clicked, this, &MainWindow::onTopicButtonClicked);
    connect(ui->serviceButton, &QPushButton::clicked, this, &MainWindow::onServiceButtonClicked);
    connect(ui->actionButton, &QPushButton::clicked, this, &MainWindow::onActionButtonClicked);
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::onTopicButtonClicked()
{
    ui->topicTreeWidget->clear();

    auto topics = node_->get_topic_names_and_types();

    for (const auto &topic : topics) {
        QString topic_name = QString::fromStdString(topic.first);
        QString types;
        for (const auto &type : topic.second) {
            types += QString::fromStdString(type) + " ";
        }

        QTreeWidgetItem *item = new QTreeWidgetItem(ui->topicTreeWidget);
        item->setText(0, topic_name);
        item->setText(1, types.trimmed());
    }
}

void MainWindow::onServiceButtonClicked()
{
    ui->serviceTreeWidget->clear();

    auto services = node_->get_service_names_and_types();

    for (const auto &service : services) {
        QString service_name = QString::fromStdString(service.first);
        QString types;
        for (const auto &type : service.second) {
            types += QString::fromStdString(type) + " ";
        }

        QTreeWidgetItem *item = new QTreeWidgetItem(ui->serviceTreeWidget);
        item->setText(0, service_name);
        item->setText(1, types.trimmed());
    }
}

void MainWindow::onActionButtonClicked()
{
    ui->actionTreeWidget->clear();

    auto topics = node_->get_topic_names_and_types();

    for (const auto &topic : topics) {
        for (const auto &type : topic.second) {
            if (type.find("action_msgs") != std::string::npos || type.find("action") != std::string::npos) {
                // Assume this topic is action related
                QString action_name = QString::fromStdString(topic.first);
                QString action_type = QString::fromStdString(type);

                QTreeWidgetItem *item = new QTreeWidgetItem(ui->actionTreeWidget);
                item->setText(0, action_name);
                item->setText(1, action_type);
            }
        }
    }
}
