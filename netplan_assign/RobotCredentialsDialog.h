#ifndef ROBOTCREDENTIALSDIALOG_H
#define ROBOTCREDENTIALSDIALOG_H

#include <QDialog>

class QLineEdit;

class RobotCredentialsDialog : public QDialog {
    Q_OBJECT
public:
    RobotCredentialsDialog(QWidget *parent = nullptr);
    QString getIp() const;
    QString getUsername() const;
    QString getPassword() const;
private:
    QLineEdit *ipEdit;
    QLineEdit *usernameEdit;
    QLineEdit *passwordEdit;
};

#endif // ROBOTCREDENTIALSDIALOG_H
