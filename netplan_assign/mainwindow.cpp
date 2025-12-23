#include <QLabel>
#include <QPropertyAnimation>
#include <QTimer>
// Ensure MainWindow is declared before any member function implementation
#include "mainwindow.h"

// Show a non-blocking notification (toast) at the bottom of the main window
void MainWindow::showNotification(const QString& message, bool success) {
    QLabel *toast = new QLabel(this);
    toast->setText(message);
    toast->setAlignment(Qt::AlignCenter);
    toast->setProperty("success", success);
    toast->setObjectName("notificationToast");
    toast->setWindowFlags(Qt::FramelessWindowHint | Qt::ToolTip);
    toast->setAttribute(Qt::WA_ShowWithoutActivating);
    toast->adjustSize();
    int x = (width() - toast->width()) / 2;
    int y = height() - toast->height() - 40;
    toast->move(x, y);
    toast->show();
    QPropertyAnimation *fade = new QPropertyAnimation(toast, "windowOpacity");
    fade->setDuration(500);
    fade->setStartValue(1.0);
    fade->setEndValue(0.0);
    QTimer::singleShot(2000, [toast, fade]() {
        fade->start();
        QObject::connect(fade, &QPropertyAnimation::finished, toast, &QLabel::deleteLater);
    });
}
#include <QInputDialog>
#include "RobotCredentialsDialog.h"
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QFile>
#include <QTimer>
#include <QPropertyAnimation>
#include <QTextStream>
#include <QFileDialog>
#include <vector>
#include <QLabel>
#include <QPalette>
#include <QList>
#include <QRegularExpressionValidator>

// Helper to request robot info once per session
bool MainWindow::requestRobotInfo() {
    if (robotInfoSet)
        return true;
    RobotCredentialsDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted)
        return false;
    robotIp = dlg.getIp();
    robotUsername = dlg.getUsername();
    robotPassword = dlg.getPassword();
    if (robotIp.isEmpty() || robotUsername.isEmpty() || robotPassword.isEmpty())
        return false;
    robotInfoSet = true;
    return true;
}
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
#include <QInputDialog>

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

    // Connect new buttons to their slots
    connect(ui->pushButton_generate_netplan, &QPushButton::clicked, this, &MainWindow::on_pushButton_generate_netplan_clicked);
    connect(ui->pushButton_try_netplan, &QPushButton::clicked, this, &MainWindow::on_pushButton_try_netplan_clicked);
// Slot: Generate Netplan button
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
    
    // Prompt for robot IP, username, and password


    if (!requestRobotInfo()) return;

    // Save to a temporary file first
    QString tempFile = "01-netcfg.yaml";
    QFile file(tempFile);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out << yamlContent;
        file.close();
    } else {
        showNotification("Failed to save temporary 01-netcfg.yaml", false);
        return;
    }

    // Use sshpass and scp to copy the file to /etc/netplan on the robot
    QString command = QString("sshpass -p '%1' scp -o StrictHostKeyChecking=no 01-netcfg.yaml %2@%3:/tmp/01-netcfg.yaml")
                        .arg(robotPassword, robotUsername, robotIp);
    int result = system(command.toStdString().c_str());
    if (result == 0) {
        // Move file to /etc/netplan with sudo
        QString moveCmd = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no %2@%3 'echo %4 | sudo -S mv /tmp/01-netcfg.yaml /etc/netplan/01-netcfg.yaml'")
                            .arg(robotPassword, robotUsername, robotIp, robotPassword);
        int moveResult = system(moveCmd.toStdString().c_str());
        if (moveResult == 0) {
            showNotification("01-netcfg.yaml copied to /etc/netplan on robot.", true);
        } else {
            showNotification("Failed to move file to /etc/netplan. Check sudo permissions on robot.", false);
        }
    } else {
        showNotification("Failed to copy file to robot. Check sshpass, network, and credentials.", false);
    }
}

// Helper: run sudo netplan generate on robot, return true if success
bool MainWindow::runNetplanGenerateOnRobot() {
    if (!requestRobotInfo()) return false;
    QString command = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no %2@%3 'echo %4 | sudo -S netplan generate'")
                        .arg(robotPassword, robotUsername, robotIp, robotPassword);
    int result = system(command.toStdString().c_str());
    return (result == 0);
}

void MainWindow::on_pushButton_generate_netplan_clicked()
{
    // ...existing code to build yamlContent if needed...
    bool ok = runNetplanGenerateOnRobot();
    if (ok) {
        showNotification("'sudo netplan generate' executed on robot.", true);
    } else {
        showNotification("Failed to run 'sudo netplan generate' on robot. Check sshpass, network, and credentials.", false);
    }
}

// Slot: Try Netplan button

// Helper: run sudo netplan try on robot, return true if success
bool MainWindow::runNetplanTryOnRobot() {
    if (!requestRobotInfo()) return false;
    QString command = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no %2@%3 'echo %4 | sudo -S netplan try'")
                        .arg(robotPassword, robotUsername, robotIp, robotPassword);
    int result = system(command.toStdString().c_str());
    return (result == 0);
}

void MainWindow::on_pushButton_try_netplan_clicked()
{
    bool ok = runNetplanTryOnRobot();
    if (ok) {
        showNotification("'sudo netplan try' executed on robot.", true);
    } else {
        showNotification("Failed to run 'sudo netplan try' on robot. Check sshpass, network, and credentials.", false);
    }
}