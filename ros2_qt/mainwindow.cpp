#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/graph_listener.hpp>
#include <QRandomGenerator>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("qt_gui_node");

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
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Unchecked);  // 0th column = checkmark
        item->setText(1, topic_name);           // 1st column = topic name
        item->setText(2, types.trimmed());       // 2nd column = type name

        int hz = QRandomGenerator::global()->bounded(0, 21);  // Random 0-20 Hz
        item->setText(3, QString::number(hz));
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
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Unchecked);
        item->setText(1, service_name);
        item->setText(2, types.trimmed());

        int hz = QRandomGenerator::global()->bounded(0, 21);
        item->setText(3, QString::number(hz));
    }
}

void MainWindow::onActionButtonClicked()
{
    ui->actionTreeWidget->clear();

    auto topics = node_->get_topic_names_and_types();

    for (const auto &topic : topics) {
        for (const auto &type : topic.second) {
            if (type.find("action_msgs") != std::string::npos || type.find("action") != std::string::npos) {
                QString action_name = QString::fromStdString(topic.first);
                QString action_type = QString::fromStdString(type);

                QTreeWidgetItem *item = new QTreeWidgetItem(ui->actionTreeWidget);
                item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
                item->setCheckState(0, Qt::Unchecked);
                item->setText(1, action_name);
                item->setText(2, action_type);

                int hz = QRandomGenerator::global()->bounded(0, 21);
                item->setText(3, QString::number(hz));
            }
        }
    }
}
