#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <vector>
#include <QLabel>
#include <QPalette>
#include <QList>
#include <QRegularExpressionValidator>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Load stylesheet from external file
    QFile styleFile(":/style.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text)) {
        QString style = styleFile.readAll();
        this->setStyleSheet(style);
        styleFile.close();
    }

    // Set IP address validator (only numbers and dots allowed)
    QRegularExpression ipRegex("^[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}$");
    QValidator *ipValidator = new QRegularExpressionValidator(ipRegex, this);

    // Apply validator to all IP input fields
    ui->Laser1_IP->setValidator(ipValidator);
    ui->Laser2_IP->setValidator(ipValidator);
    ui->MiniPc_IP->setValidator(ipValidator);
    ui->Android_IP->setValidator(ipValidator);
    ui->reserved1_IP->setValidator(ipValidator);
    ui->reserved2_IP->setValidator(ipValidator);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_generate_clicked()
{
    // Create YAML content
    QString yamlContent = "network:\n";
    yamlContent += "  version: 2\n";
    yamlContent += "  ethernets:\n";
    
    // Add each device configuration if both interface and IP are provided
    struct DeviceField {
        QLineEdit* iface;
        QLineEdit* ip;
        QLabel* label;
    };
    std::vector<std::pair<QString, DeviceField>> devices = {
        {"Laser1", {ui->Laser1_hardware, ui->Laser1_IP, ui->Laser1}},
        {"Laser2", {ui->Laser2_hardware, ui->Laser2_IP, ui->Laser2}},
        {"MiniPc", {ui->MiniPc_hardware, ui->MiniPc_IP, ui->MiniPc}},
        {"Android", {ui->Android_hardware, ui->Android_IP, ui->Android}},
        {"reserved1", {ui->reserved1_hardware, ui->reserved1_IP, ui->reserved1}},
        {"reserved2", {ui->reserved2_hardware, ui->reserved2_IP, ui->reserved2}}
    };

    auto sanitize = [](const QString &s)->QString {
        QString t = s.trimmed();
        t.replace(' ', '_');
        return t.toLower();
    };

    for (const auto& dev : devices) {
        QString ifaceInput = dev.second.iface->text().trimmed();
        QString ip = dev.second.ip->text().trimmed();
        QString labelText;
        if (dev.second.label)
            labelText = dev.second.label->text().trimmed();

        if (!ifaceInput.isEmpty() && !ip.isEmpty()) {
            QString iface = sanitize(ifaceInput);
            // generate gateway as .1 of the given IP (common default)
            QString gateway;
            QStringList parts = ip.split('.');
            if (parts.size() >= 4) {
                parts[3] = "1";
                gateway = parts.join('.');
            }

            // add a blank line and the label comment before each iface block
            yamlContent += "\n";
            if (!labelText.isEmpty()) {
                yamlContent += "    # " + labelText + "\n";
            }
            yamlContent += "    " + iface + ":\n";
            yamlContent += "      dhcp4: no\n";
            yamlContent += "      addresses:\n";
            yamlContent += "        - " + ip + "/24\n";
            // if (!gateway.isEmpty()) {
            //     yamlContent += "      gateway4: " + gateway + "   # optional if you have internet on this subnet\n";
            // }
            // yamlContent += "      nameservers:\n";
            // yamlContent += "        addresses: [8.8.8.8, 1.1.1.1]   # optional\n";
        }
    }
    
    // Save to file
    QString fileName = QFileDialog::getSaveFileName(this,
                                                     "Save Network Configuration",
                                                     "netcfg.yaml",
                                                     "YAML Files (*.yaml *.yml);;All Files (*)");
    
    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            out << yamlContent;
            file.close();
            QMessageBox::information(this, "Success", "Network configuration saved to:\n" + fileName);
        } else {
            QMessageBox::critical(this, "Error", "Failed to save file:\n" + fileName);
        }
    }
}
