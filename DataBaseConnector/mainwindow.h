#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QNetworkAccessManager>
#include <QNetworkReply>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onConnectButtonClicked();
    void onApiReplyFinished();

private:
    Ui::MainWindow *ui;
    QNetworkAccessManager *networkManager;
    QNetworkReply *currentReply;
};
#endif // MAINWINDOW_H
