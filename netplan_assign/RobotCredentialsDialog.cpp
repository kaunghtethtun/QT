#include "RobotCredentialsDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>

RobotCredentialsDialog::RobotCredentialsDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("Robot Connection Info");
    QVBoxLayout *mainLayout = new QVBoxLayout(this);


    QHBoxLayout *ipLayout = new QHBoxLayout;
    QLabel *ipLabel = new QLabel("IP Address:");
    ipLayout->addWidget(ipLabel);
    ipEdit = new QLineEdit;
    ipEdit->setPlaceholderText("e.g. 192.168.1.100");
    ipLayout->addWidget(ipEdit);
    mainLayout->addLayout(ipLayout);

    QHBoxLayout *userLayout = new QHBoxLayout;
    QLabel *userLabel = new QLabel("Username:");
    userLayout->addWidget(userLabel);
    usernameEdit = new QLineEdit;
    usernameEdit->setPlaceholderText("e.g. robot");
    userLayout->addWidget(usernameEdit);
    mainLayout->addLayout(userLayout);

    QHBoxLayout *passLayout = new QHBoxLayout;
    QLabel *passLabel = new QLabel("Password:");
    passLayout->addWidget(passLabel);
    passwordEdit = new QLineEdit;
    passwordEdit->setEchoMode(QLineEdit::Password);
    passLayout->addWidget(passwordEdit);
    mainLayout->addLayout(passLayout);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);
}

QString RobotCredentialsDialog::getIp() const { return ipEdit->text(); }
QString RobotCredentialsDialog::getUsername() const { return usernameEdit->text(); }
QString RobotCredentialsDialog::getPassword() const { return passwordEdit->text(); }
