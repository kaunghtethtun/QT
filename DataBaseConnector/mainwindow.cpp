#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QJsonObject>
#include <QJsonDocument>
#include <QUrl>
#include <QNetworkRequest>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      networkManager(new QNetworkAccessManager(this)), currentReply(nullptr) {
    ui->setupUi(this);
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::onConnectButtonClicked() {
    QJsonObject json;
    json["host"] = ui->lineEditHost->text();
    json["port"] = ui->lineEditPort->text().toInt();
    json["dbName"] = ui->lineEditDb->text();
    json["username"] = ui->lineEditUser->text();
    json["password"] = ui->lineEditPass->text();
    json["dbType"] = ui->lineEditType->text();

    QUrl url("http://localhost:5000/databaseconnrequest"); // change if remote
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    currentReply = networkManager->post(request, QJsonDocument(json).toJson());
    connect(currentReply, &QNetworkReply::finished, this, &MainWindow::onApiReplyFinished);

    ui->textEditResponse->setText("Connecting to API...");
}

void MainWindow::onApiReplyFinished() {
    if (!currentReply)
        return;

    QByteArray response = currentReply->readAll();
    if (currentReply->error() == QNetworkReply::NoError) {
        QJsonDocument doc = QJsonDocument::fromJson(response);
        QJsonObject obj = doc.object();
        QString status = obj["status"].toString();
        QString msg = obj["message"].toString();
        ui->textEditResponse->setText("Status: " + status + "\nMessage: " + msg);
    } else {
        ui->textEditResponse->setText("Error: " + currentReply->errorString());
    }

    currentReply->deleteLater();
    currentReply = nullptr;
}
