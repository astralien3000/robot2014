/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.0.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *connect_action;
    QWidget *main_widget;
    QHBoxLayout *horizontalLayout;
    QSplitter *splitter;
    QWidget *left_pannel;
    QVBoxLayout *left_layout;
    QSplitter *splitter_2;
    QGraphicsView *screen;
    QTextEdit *console;
    QTreeWidget *system_view;
    QMenuBar *menu;
    QMenu *file_menu;
    QStatusBar *status_bar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(800, 600);
        connect_action = new QAction(MainWindow);
        connect_action->setObjectName(QStringLiteral("connect_action"));
        main_widget = new QWidget(MainWindow);
        main_widget->setObjectName(QStringLiteral("main_widget"));
        horizontalLayout = new QHBoxLayout(main_widget);
        horizontalLayout->setSpacing(2);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(4, 4, 4, 4);
        splitter = new QSplitter(main_widget);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        left_pannel = new QWidget(splitter);
        left_pannel->setObjectName(QStringLiteral("left_pannel"));
        left_layout = new QVBoxLayout(left_pannel);
        left_layout->setObjectName(QStringLiteral("left_layout"));
        left_layout->setContentsMargins(0, 0, 0, 0);
        splitter_2 = new QSplitter(left_pannel);
        splitter_2->setObjectName(QStringLiteral("splitter_2"));
        splitter_2->setOrientation(Qt::Vertical);
        screen = new QGraphicsView(splitter_2);
        screen->setObjectName(QStringLiteral("screen"));
        splitter_2->addWidget(screen);
        console = new QTextEdit(splitter_2);
        console->setObjectName(QStringLiteral("console"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(console->sizePolicy().hasHeightForWidth());
        console->setSizePolicy(sizePolicy);
        splitter_2->addWidget(console);

        left_layout->addWidget(splitter_2);

        splitter->addWidget(left_pannel);
        system_view = new QTreeWidget(splitter);
        new QTreeWidgetItem(system_view);
        system_view->setObjectName(QStringLiteral("system_view"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(system_view->sizePolicy().hasHeightForWidth());
        system_view->setSizePolicy(sizePolicy1);
        system_view->setSelectionMode(QAbstractItemView::SingleSelection);
        system_view->setTextElideMode(Qt::ElideRight);
        system_view->setAutoExpandDelay(-1);
        system_view->setIndentation(20);
        system_view->setUniformRowHeights(false);
        system_view->setSortingEnabled(false);
        system_view->setAnimated(false);
        system_view->setAllColumnsShowFocus(false);
        system_view->setWordWrap(true);
        system_view->setHeaderHidden(false);
        system_view->setExpandsOnDoubleClick(true);
        system_view->setColumnCount(2);
        splitter->addWidget(system_view);
        system_view->header()->setCascadingSectionResizes(false);
        system_view->header()->setDefaultSectionSize(150);
        system_view->header()->setHighlightSections(true);
        system_view->header()->setMinimumSectionSize(50);
        system_view->header()->setProperty("showSortIndicator", QVariant(false));
        system_view->header()->setStretchLastSection(true);

        horizontalLayout->addWidget(splitter);

        MainWindow->setCentralWidget(main_widget);
        menu = new QMenuBar(MainWindow);
        menu->setObjectName(QStringLiteral("menu"));
        menu->setGeometry(QRect(0, 0, 800, 25));
        file_menu = new QMenu(menu);
        file_menu->setObjectName(QStringLiteral("file_menu"));
        MainWindow->setMenuBar(menu);
        status_bar = new QStatusBar(MainWindow);
        status_bar->setObjectName(QStringLiteral("status_bar"));
        MainWindow->setStatusBar(status_bar);

        menu->addAction(file_menu->menuAction());
        file_menu->addAction(connect_action);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Eirbot IHM", 0));
        connect_action->setText(QApplication::translate("MainWindow", "Connect", 0));
        QTreeWidgetItem *___qtreewidgetitem = system_view->headerItem();
        ___qtreewidgetitem->setText(1, QApplication::translate("MainWindow", "Value", 0));
        ___qtreewidgetitem->setText(0, QApplication::translate("MainWindow", "Device", 0));

        const bool __sortingEnabled = system_view->isSortingEnabled();
        system_view->setSortingEnabled(false);
        QTreeWidgetItem *___qtreewidgetitem1 = system_view->topLevelItem(0);
        ___qtreewidgetitem1->setText(1, QApplication::translate("MainWindow", "1", 0));
        ___qtreewidgetitem1->setText(0, QApplication::translate("MainWindow", "test", 0));
        system_view->setSortingEnabled(__sortingEnabled);

        file_menu->setTitle(QApplication::translate("MainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
