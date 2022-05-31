# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/control.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowModality(QtCore.Qt.NonModal)
        MainWindow.setEnabled(True)
        MainWindow.resize(719, 597)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("ui/../icon/bot.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.gridLayout.setContentsMargins(0, -1, -1, -1)
        self.gridLayout.setObjectName("gridLayout")
        self.gridLayout.setRowStretch(0, 3)
        self.gridLayout_3.addLayout(self.gridLayout, 0, 0, 1, 3)
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setTitle("")
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_4)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.nav_start_button = QtWidgets.QPushButton(self.groupBox_4)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("ui/../icon/marker.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.nav_start_button.setIcon(icon1)
        self.nav_start_button.setObjectName("nav_start_button")
        self.verticalLayout_3.addWidget(self.nav_start_button)
        self.yolo_onoff_button = QtWidgets.QPushButton(self.groupBox_4)
        self.yolo_onoff_button.setObjectName("yolo_onoff_button")
        self.verticalLayout_3.addWidget(self.yolo_onoff_button)
        self.gridLayout_3.addWidget(self.groupBox_4, 2, 1, 1, 1)
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setObjectName("widget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.interrupt_button = QtWidgets.QPushButton(self.widget)
        self.interrupt_button.setAutoFillBackground(False)
        self.interrupt_button.setStyleSheet("border: 0px;")
        self.interrupt_button.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("ui/../icon/stop-button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.interrupt_button.setIcon(icon2)
        self.interrupt_button.setIconSize(QtCore.QSize(90, 90))
        self.interrupt_button.setShortcut("")
        self.interrupt_button.setCheckable(False)
        self.interrupt_button.setObjectName("interrupt_button")
        self.verticalLayout.addWidget(self.interrupt_button)
        self.gridLayout_3.addWidget(self.widget, 1, 0, 2, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 719, 22))
        self.menubar.setTabletTracking(False)
        self.menubar.setObjectName("menubar")
        self.menuView = QtWidgets.QMenu(self.menubar)
        self.menuView.setObjectName("menuView")
        self.menuGripper = QtWidgets.QMenu(self.menubar)
        self.menuGripper.setObjectName("menuGripper")
        self.menuTM_Pos = QtWidgets.QMenu(self.menubar)
        self.menuTM_Pos.setObjectName("menuTM_Pos")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionNode_checker = QtWidgets.QAction(MainWindow)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("ui/../icon/list-check.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionNode_checker.setIcon(icon3)
        self.actionNode_checker.setObjectName("actionNode_checker")
        self.actionAmm_control = QtWidgets.QAction(MainWindow)
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("ui/../icon/car.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionAmm_control.setIcon(icon4)
        self.actionAmm_control.setObjectName("actionAmm_control")
        self.actionOpen = QtWidgets.QAction(MainWindow)
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap("ui/../icon/hello.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionOpen.setIcon(icon5)
        self.actionOpen.setObjectName("actionOpen")
        self.actionClose = QtWidgets.QAction(MainWindow)
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap("ui/../icon/letter-s.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionClose.setIcon(icon6)
        self.actionClose.setObjectName("actionClose")
        self.actionReset = QtWidgets.QAction(MainWindow)
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap("ui/../icon/refresh.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionReset.setIcon(icon7)
        self.actionReset.setObjectName("actionReset")
        self.actionActive = QtWidgets.QAction(MainWindow)
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap("ui/../icon/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionActive.setIcon(icon8)
        self.actionActive.setObjectName("actionActive")
        self.actionArm_control = QtWidgets.QAction(MainWindow)
        icon9 = QtGui.QIcon()
        icon9.addPixmap(QtGui.QPixmap("ui/../icon/robot-arm.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionArm_control.setIcon(icon9)
        self.actionArm_control.setObjectName("actionArm_control")
        self.actionInit_Pos = QtWidgets.QAction(MainWindow)
        self.actionInit_Pos.setObjectName("actionInit_Pos")
        self.actionYolo_Pos = QtWidgets.QAction(MainWindow)
        self.actionYolo_Pos.setObjectName("actionYolo_Pos")
        self.menuView.addAction(self.actionNode_checker)
        self.menuView.addAction(self.actionAmm_control)
        self.menuView.addAction(self.actionArm_control)
        self.menuGripper.addAction(self.actionActive)
        self.menuGripper.addAction(self.actionReset)
        self.menuGripper.addAction(self.actionOpen)
        self.menuGripper.addAction(self.actionClose)
        self.menuTM_Pos.addAction(self.actionInit_Pos)
        self.menuTM_Pos.addAction(self.actionYolo_Pos)
        self.menubar.addAction(self.menuView.menuAction())
        self.menubar.addAction(self.menuGripper.menuAction())
        self.menubar.addAction(self.menuTM_Pos.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Medicare"))
        self.nav_start_button.setText(_translate("MainWindow", "Navigation"))
        self.yolo_onoff_button.setText(_translate("MainWindow", "Yolo On/Off"))
        self.menuView.setTitle(_translate("MainWindow", "View"))
        self.menuGripper.setTitle(_translate("MainWindow", "Gripper"))
        self.menuTM_Pos.setTitle(_translate("MainWindow", "TM_Pos"))
        self.actionNode_checker.setText(_translate("MainWindow", "Node checker"))
        self.actionAmm_control.setText(_translate("MainWindow", "Amm control"))
        self.actionOpen.setText(_translate("MainWindow", "Open"))
        self.actionClose.setText(_translate("MainWindow", "Close"))
        self.actionReset.setText(_translate("MainWindow", "Reset"))
        self.actionActive.setText(_translate("MainWindow", "Active"))
        self.actionArm_control.setText(_translate("MainWindow", "Arm control"))
        self.actionInit_Pos.setText(_translate("MainWindow", "Init_Pos"))
        self.actionYolo_Pos.setText(_translate("MainWindow", "Yolo_Pos"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
