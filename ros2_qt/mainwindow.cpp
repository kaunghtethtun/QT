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

        QTreeWidgetItem *parentItem = new QTreeWidgetItem(ui->topicTreeWidget);
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsUserCheckable);
        parentItem->setCheckState(0, Qt::Unchecked);
        parentItem->setText(1, topic_name);
        parentItem->setText(2, types.trimmed());

        //int hz = QRandomGenerator::global()->bounded(0, 21);
        parentItem->setText(3, "0");
        
        // 👉 Make Hz column editable
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsEditable);

        // 👉 Add a child item
        QTreeWidgetItem *childItem = new QTreeWidgetItem(parentItem);
        childItem->setText(1, "value");
        childItem->setFlags(childItem->flags() | Qt::ItemIsEditable);

        parentItem->setExpanded(true);
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

        QTreeWidgetItem *parentItem = new QTreeWidgetItem(ui->serviceTreeWidget);
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsUserCheckable);
        parentItem->setCheckState(0, Qt::Unchecked);
        parentItem->setText(1, service_name);
        parentItem->setText(2, types.trimmed());

        //int hz = QRandomGenerator::global()->bounded(0, 21);
        parentItem->setText(3, "0");

        // 👉 Make Hz column editable
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsEditable);

        // 👉 Add a child item
        QTreeWidgetItem *childItem = new QTreeWidgetItem(parentItem);
        childItem->setText(1, "Editable Request");
        childItem->setFlags(childItem->flags() | Qt::ItemIsEditable);

        parentItem->setExpanded(true);
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

                QTreeWidgetItem *parentItem = new QTreeWidgetItem(ui->actionTreeWidget);
                parentItem->setFlags(parentItem->flags() | Qt::ItemIsUserCheckable);
                parentItem->setCheckState(0, Qt::Unchecked);
                parentItem->setText(1, action_name);
                parentItem->setText(2, action_type);

                //int hz = QRandomGenerator::global()->bounded(0, 21);
                parentItem->setText(3, "0");

                // 👉 Make Hz column editable
                parentItem->setFlags(parentItem->flags() | Qt::ItemIsEditable);

                // 👉 Add a child item
                QTreeWidgetItem *childItem = new QTreeWidgetItem(parentItem);
                childItem->setText(1, "Editable Goal");
                childItem->setFlags(childItem->flags() | Qt::ItemIsEditable);

                parentItem->setExpanded(true);
            }
        }
    }
}

