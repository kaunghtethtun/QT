#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QComboBox>
#include <QStringList>
#include <QSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QDebug>

QMap<int, QStringList> comboOptions = {
    {0, {"Motor", "Motor 1 (120kg)", "Motor 2 (300kg)"}},
    {1, {"Motor Driver", "L2DB4830-CAFR", "L2DB4830-CAFC" }},
    {2, {"IMU", "LIDAR", "IMU", "Ultrasonic"}},
    {3, {"LIDAR", "Rubber Wheel", "Omni Wheel"}},
    {4, {"Frame", "Aluminum Frame", "Steel Frame"}},
    {5, {"Controller", "STM32", "Arduino", "Raspberry Pi"}},
    {6, {"Power Supply", "12V 5A", "24V 5A"}},
    {7, {"Battery", "Battery 24V 20Ah", "Battery 24V 40Ah"}},
    {8, {"Wire Set", "Signal Wires", "Power Cables"}},
    {9, {"Mount", "Bracket A", "Bracket B"}}
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int itemRows = 10;                      // 10 items
    int totalRow = itemRows;               // 11th row for total
    ui->tableWidget->setRowCount(itemRows + 1); // Add 1 for total cost row

    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setColumnWidth(0, 10);
    ui->tableWidget->setColumnWidth(1, 200);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    ui->tableWidget->setColumnWidth(3, 80);
    ui->tableWidget->setColumnWidth(4, 200);
    ui->tableWidget->setColumnWidth(5, 200);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);

    for(int row = 0; row <= itemRows; ++row) {
        // ui->tableWidget->verticalHeader()->setSectionResizeMode(row, QHeaderView::Stretch);
        ui->tableWidget->setRowHeight(row, 150); // Set a fixed height for each row
    }

    for (int row = 0; row < itemRows; ++row) {
        ui->tableWidget->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));

        QComboBox *combo = new QComboBox(this);
        combo->addItems(comboOptions[row]);
        ui->tableWidget->setCellWidget(row, 1, combo);

        QWidget *descWidget = new QWidget();
        QLabel *imageLabel = new QLabel();
        QLabel *descLabel = new QLabel("Select an item");
        QHBoxLayout *layout = new QHBoxLayout(descWidget);
        layout->addWidget(imageLabel);
        layout->addWidget(descLabel);
        layout->setContentsMargins(0, 0, 0, 0);
        descWidget->setLayout(layout);
        ui->tableWidget->setCellWidget(row, 2, descWidget);

        QSpinBox *spinBox = new QSpinBox(this);
        spinBox->setRange(0, 100);
        ui->tableWidget->setCellWidget(row, 3, spinBox);

        QTableWidgetItem *unitPriceItem = new QTableWidgetItem("0");
        unitPriceItem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(row, 4, unitPriceItem);

        QTableWidgetItem *totalItem = new QTableWidgetItem("0");
        totalItem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(row, 5, totalItem);

        // Update total cost row
        auto updateTotalCost = [=]() {
            int grandTotal = 0;
            for (int r = 0; r < itemRows; ++r) {
                grandTotal += ui->tableWidget->item(r, 5)->text().toInt();
            }
            QTableWidgetItem *grandtotal = new QTableWidgetItem(QString::number(grandTotal));
            grandtotal->setTextAlignment(Qt::AlignCenter);
            grandtotal->setBackground(Qt::yellow);
            ui->tableWidget->setItem(totalRow, 5, grandtotal);
        };

        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
            int unitPrice = ui->tableWidget->item(row, 4)->text().toInt();
            int total = unitPrice * value;
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total));
            totalItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 5, totalItem);
            //ui->tableWidget->setItem(row, 5, new QTableWidgetItem(QString::number(total)));
            updateTotalCost();
        });

        connect(combo, &QComboBox::currentTextChanged, this, [=](const QString &text){
            QString imagePath;
            QString description;
            int unitPrice = 0;

            if (text.contains("Motor 1")) {
                imagePath = ":/images/motor1.jpg";
                description = "Motor 1 - 120kg, 24V DC";
                unitPrice = 149;
            } else if (text.contains("Motor 2")) {
                imagePath = ":/images/motor2.jpg";
                description = "Motor 2 - 300kg, 24V DC";
                unitPrice = 210;
            } else if (text.contains("L2DB4830-CAFR")) {
                imagePath = ":/images/motordriver1.png";
                description = "Communication Uart+Can (115200bps,1Mbps)";
                unitPrice = 175;
            } else if (text.contains("L2DB4830-CAFC")) {
                imagePath = ":/images/motordriver1.png";
                description = "Communication Uart+RS485 (115200bps)";
                unitPrice = 175;
            } else if (text.contains("Battery 24V 20Ah")) {
                imagePath = ":/images/battery1.jpeg";
                description = "24V 20Ah LiPo4 Battery";
                unitPrice = 150;
            } else if (text.contains("Battery 24V 40Ah")) {
                imagePath = ":/images/battery2.jpg";
                description = "24V 40Ah LiPo4 Battery";
                unitPrice = 120;
            } else {
                description = "Select an item";
            }

            imageLabel->setPixmap(QPixmap(imagePath).scaled(200, 200, Qt::KeepAspectRatio));
            descLabel->setText(description);

            QTableWidgetItem *unitPriceItem = new QTableWidgetItem(QString::number(unitPrice));
            unitPriceItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 4, unitPriceItem);

            int quantity = spinBox->value();
            int total = quantity * unitPrice;
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total));
            totalItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 5, totalItem);
            updateTotalCost();
        });
    }

    // Setup Total Cost row
    QTableWidgetItem *labelItem = new QTableWidgetItem("Total Cost:");
    // labelItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    // ui->tableWidget->setItem(totalRow, 0, labelItem);
    QFont font = labelItem->font();
    font.setBold(true);                    // Make it bold
    labelItem->setFont(font);
    labelItem->setForeground(QBrush(Qt::blue));  // Set text color (e.g., blue)
    labelItem->setTextAlignment(Qt::AlignCenter);  // Center the text
    labelItem->setBackground(Qt::yellow);
// Set item and merge columns 0 to 4
    ui->tableWidget->setSpan(totalRow, 0, 1, 5);  // Merge columns 0 to 4
    ui->tableWidget->setItem(totalRow, 0, labelItem);
    QTableWidgetItem *totalCostItem = new QTableWidgetItem("0");
    totalCostItem->setTextAlignment(Qt::AlignCenter);
    totalCostItem->setBackground(Qt::yellow);
    ui->tableWidget->setItem(totalRow, 5, totalCostItem);
    //ui->tableWidget->setItem(totalRow, 5, new QTableWidgetItem("0"));
    ui->tableWidget->setSpan(totalRow, 0, 1, 5); // Merge columns 0 to 3
}

MainWindow::~MainWindow()
{
    delete ui;
}
