#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QComboBox>
#include <QStringList>
#include <QSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setColumnWidth(0, 10); 
    ui->tableWidget->setColumnWidth(1, 200); 
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    ui->tableWidget->setColumnWidth(3, 80); 
    ui->tableWidget->setColumnWidth(4, 200); 
    ui->tableWidget->setColumnWidth(5, 200); 
    ui->tableWidget->setColumnWidth(6, 200); 
    
    
    for(int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        ui->tableWidget->verticalHeader()->setSectionResizeMode(row, QHeaderView::Stretch); 
    }
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        QTableWidgetItem* item = new QTableWidgetItem(QString::number(row+1));
        ui->tableWidget->setItem(row, 0, item);
    }
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) 
    {
        QSpinBox *spinBox = new QSpinBox(this);
        spinBox->setMinimum(0);
        spinBox->setMaximum(100);
        spinBox->setValue(0); // default value

        ui->tableWidget->setCellWidget(row, 3, spinBox); // column 2

        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
        QTableWidgetItem *unitPriceItem = ui->tableWidget->item(row, 4);
        if (unitPriceItem) {
            int unitPrice = unitPriceItem->text().toInt();
            int total = unitPrice * value;
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total));
            ui->tableWidget->setItem(row, 5, totalItem);
        }
    });
    }
    // Combo box options
    QStringList comboItems = {"Motor", "Motor 1(120kg)", "Motor 2(300kg)"};

    // Insert combo boxes into all cells of column 1
    QComboBox *combo = new QComboBox(this);
    combo->addItems(comboItems);
    ui->tableWidget->setCellWidget(0, 1, combo);

    connect(combo, &QComboBox::currentTextChanged, this, [=](const QString &text){
        QWidget *widget = new QWidget();
        QHBoxLayout *layout = new QHBoxLayout(widget);
        QLabel *iconLabel = new QLabel();
        QLabel *descLabel = new QLabel();
    
        // Choose image and description based on selection
        if (text == "Motor 1(120kg)") {
            iconLabel->setPixmap(QPixmap(":/images/motor1.jpg").scaled(150, 150, Qt::KeepAspectRatio));
            descLabel->setText("Heavy-duty motor 120kg 24V DC Uart,Can bus");
            QTableWidgetItem* price = new QTableWidgetItem(QString::number(149));
            ui->tableWidget->setItem(0, 4, price);
        } else if (text == "Motor 2(300kg)") {
            iconLabel->setPixmap(QPixmap(":/images/motor2.jpg").scaled(150, 150, Qt::KeepAspectRatio));
            descLabel->setText("Ultra heavy-duty 300kg 24V DC Uart,Can bus");
            QTableWidgetItem* price = new QTableWidgetItem(QString::number(210));
            ui->tableWidget->setItem(0, 4, price);
        }

        descLabel->setWordWrap(true);
        layout->addWidget(iconLabel);
        layout->addWidget(descLabel);
        layout->setContentsMargins(0, 0, 0, 0);
        widget->setLayout(layout);

        ui->tableWidget->setCellWidget(0, 2, widget); // set in column 2
    });

}

MainWindow::~MainWindow()
{
    delete ui;
}

