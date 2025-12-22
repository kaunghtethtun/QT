/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QLabel *titleLabel;
    QGridLayout *gridLayout;
    QLabel *Laser1;
    QLineEdit *Laser1_hardware;
    QLineEdit *Laser1_IP;
    QLabel *Laser2;
    QLineEdit *Laser2_hardware;
    QLineEdit *Laser2_IP;
    QLabel *MiniPc;
    QLineEdit *MiniPc_hardware;
    QLineEdit *MiniPc_IP;
    QLabel *Android;
    QLineEdit *Android_hardware;
    QLineEdit *Android_IP;
    QLabel *reserved1;
    QLineEdit *reserved1_hardware;
    QLineEdit *reserved1_IP;
    QLabel *reserved2;
    QLineEdit *reserved2_hardware;
    QLineEdit *reserved2_IP;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_generate;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 463);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName("verticalLayout");
        titleLabel = new QLabel(centralwidget);
        titleLabel->setObjectName("titleLabel");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(titleLabel->sizePolicy().hasHeightForWidth());
        titleLabel->setSizePolicy(sizePolicy);
        QFont font;
        font.setPointSize(18);
        font.setBold(true);
        titleLabel->setFont(font);
        titleLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout->addWidget(titleLabel);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        gridLayout->setHorizontalSpacing(10);
        gridLayout->setVerticalSpacing(15);
        Laser1 = new QLabel(centralwidget);
        Laser1->setObjectName("Laser1");
        Laser1->setMinimumSize(QSize(0, 40));
        QFont font1;
        font1.setPointSize(12);
        Laser1->setFont(font1);

        gridLayout->addWidget(Laser1, 0, 0, 1, 1);

        Laser1_hardware = new QLineEdit(centralwidget);
        Laser1_hardware->setObjectName("Laser1_hardware");
        Laser1_hardware->setMinimumSize(QSize(0, 40));
        QFont font2;
        font2.setPointSize(11);
        Laser1_hardware->setFont(font2);

        gridLayout->addWidget(Laser1_hardware, 0, 1, 1, 1);

        Laser1_IP = new QLineEdit(centralwidget);
        Laser1_IP->setObjectName("Laser1_IP");
        Laser1_IP->setMinimumSize(QSize(0, 40));
        Laser1_IP->setFont(font2);

        gridLayout->addWidget(Laser1_IP, 0, 2, 1, 1);

        Laser2 = new QLabel(centralwidget);
        Laser2->setObjectName("Laser2");
        Laser2->setMinimumSize(QSize(0, 40));
        Laser2->setFont(font1);

        gridLayout->addWidget(Laser2, 1, 0, 1, 1);

        Laser2_hardware = new QLineEdit(centralwidget);
        Laser2_hardware->setObjectName("Laser2_hardware");
        Laser2_hardware->setMinimumSize(QSize(0, 40));
        Laser2_hardware->setFont(font2);

        gridLayout->addWidget(Laser2_hardware, 1, 1, 1, 1);

        Laser2_IP = new QLineEdit(centralwidget);
        Laser2_IP->setObjectName("Laser2_IP");
        Laser2_IP->setMinimumSize(QSize(0, 40));
        Laser2_IP->setFont(font2);

        gridLayout->addWidget(Laser2_IP, 1, 2, 1, 1);

        MiniPc = new QLabel(centralwidget);
        MiniPc->setObjectName("MiniPc");
        MiniPc->setMinimumSize(QSize(0, 40));
        MiniPc->setFont(font1);

        gridLayout->addWidget(MiniPc, 2, 0, 1, 1);

        MiniPc_hardware = new QLineEdit(centralwidget);
        MiniPc_hardware->setObjectName("MiniPc_hardware");
        MiniPc_hardware->setMinimumSize(QSize(0, 40));
        MiniPc_hardware->setFont(font2);

        gridLayout->addWidget(MiniPc_hardware, 2, 1, 1, 1);

        MiniPc_IP = new QLineEdit(centralwidget);
        MiniPc_IP->setObjectName("MiniPc_IP");
        MiniPc_IP->setMinimumSize(QSize(0, 40));
        MiniPc_IP->setFont(font2);

        gridLayout->addWidget(MiniPc_IP, 2, 2, 1, 1);

        Android = new QLabel(centralwidget);
        Android->setObjectName("Android");
        Android->setMinimumSize(QSize(0, 40));
        Android->setFont(font1);

        gridLayout->addWidget(Android, 3, 0, 1, 1);

        Android_hardware = new QLineEdit(centralwidget);
        Android_hardware->setObjectName("Android_hardware");
        Android_hardware->setMinimumSize(QSize(0, 40));
        Android_hardware->setFont(font2);

        gridLayout->addWidget(Android_hardware, 3, 1, 1, 1);

        Android_IP = new QLineEdit(centralwidget);
        Android_IP->setObjectName("Android_IP");
        Android_IP->setMinimumSize(QSize(0, 40));
        Android_IP->setFont(font2);

        gridLayout->addWidget(Android_IP, 3, 2, 1, 1);

        reserved1 = new QLabel(centralwidget);
        reserved1->setObjectName("reserved1");
        reserved1->setMinimumSize(QSize(0, 40));
        reserved1->setFont(font1);

        gridLayout->addWidget(reserved1, 4, 0, 1, 1);

        reserved1_hardware = new QLineEdit(centralwidget);
        reserved1_hardware->setObjectName("reserved1_hardware");
        reserved1_hardware->setMinimumSize(QSize(0, 40));
        reserved1_hardware->setFont(font2);

        gridLayout->addWidget(reserved1_hardware, 4, 1, 1, 1);

        reserved1_IP = new QLineEdit(centralwidget);
        reserved1_IP->setObjectName("reserved1_IP");
        reserved1_IP->setMinimumSize(QSize(0, 40));
        reserved1_IP->setFont(font2);

        gridLayout->addWidget(reserved1_IP, 4, 2, 1, 1);

        reserved2 = new QLabel(centralwidget);
        reserved2->setObjectName("reserved2");
        reserved2->setMinimumSize(QSize(0, 40));
        reserved2->setFont(font1);

        gridLayout->addWidget(reserved2, 5, 0, 1, 1);

        reserved2_hardware = new QLineEdit(centralwidget);
        reserved2_hardware->setObjectName("reserved2_hardware");
        reserved2_hardware->setMinimumSize(QSize(0, 40));
        reserved2_hardware->setFont(font2);

        gridLayout->addWidget(reserved2_hardware, 5, 1, 1, 1);

        reserved2_IP = new QLineEdit(centralwidget);
        reserved2_IP->setObjectName("reserved2_IP");
        reserved2_IP->setMinimumSize(QSize(0, 40));
        reserved2_IP->setFont(font2);

        gridLayout->addWidget(reserved2_IP, 5, 2, 1, 1);


        verticalLayout->addLayout(gridLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButton_generate = new QPushButton(centralwidget);
        pushButton_generate->setObjectName("pushButton_generate");
        pushButton_generate->setMinimumSize(QSize(150, 50));
        QFont font3;
        font3.setPointSize(12);
        font3.setBold(true);
        pushButton_generate->setFont(font3);

        horizontalLayout->addWidget(pushButton_generate);


        verticalLayout->addLayout(horizontalLayout);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "NetPlan Assign", nullptr));
        titleLabel->setText(QCoreApplication::translate("MainWindow", "NetPlan Assign", nullptr));
        Laser1->setText(QCoreApplication::translate("MainWindow", "Laser1", nullptr));
        Laser1_hardware->setPlaceholderText(QCoreApplication::translate("MainWindow", "Hardware Status", nullptr));
        Laser1_IP->setPlaceholderText(QCoreApplication::translate("MainWindow", "IP Address", nullptr));
        Laser2->setText(QCoreApplication::translate("MainWindow", "Laser2", nullptr));
        Laser2_hardware->setPlaceholderText(QCoreApplication::translate("MainWindow", "Hardware Status", nullptr));
        Laser2_IP->setPlaceholderText(QCoreApplication::translate("MainWindow", "IP Address", nullptr));
        MiniPc->setText(QCoreApplication::translate("MainWindow", "MiniPc", nullptr));
        MiniPc_hardware->setPlaceholderText(QCoreApplication::translate("MainWindow", "Hardware Status", nullptr));
        MiniPc_IP->setPlaceholderText(QCoreApplication::translate("MainWindow", "IP Address", nullptr));
        Android->setText(QCoreApplication::translate("MainWindow", "Android", nullptr));
        Android_hardware->setPlaceholderText(QCoreApplication::translate("MainWindow", "Hardware Status", nullptr));
        Android_IP->setPlaceholderText(QCoreApplication::translate("MainWindow", "IP Address", nullptr));
        reserved1->setText(QCoreApplication::translate("MainWindow", "reserved", nullptr));
        reserved1_hardware->setPlaceholderText(QCoreApplication::translate("MainWindow", "Hardware Status", nullptr));
        reserved1_IP->setPlaceholderText(QCoreApplication::translate("MainWindow", "IP Address", nullptr));
        reserved2->setText(QCoreApplication::translate("MainWindow", "reserved", nullptr));
        reserved2_hardware->setPlaceholderText(QCoreApplication::translate("MainWindow", "Hardware Status", nullptr));
        reserved2_IP->setPlaceholderText(QCoreApplication::translate("MainWindow", "IP Address", nullptr));
        pushButton_generate->setText(QCoreApplication::translate("MainWindow", "Generate", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
