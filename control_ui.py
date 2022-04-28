# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'control.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1600, 1600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(40, 30, 181, 261))
        self.groupBox.setObjectName("groupBox")
        self.aruco_on = QtWidgets.QPushButton(self.groupBox)
        self.aruco_on.setGeometry(QtCore.QRect(20, 50, 131, 61))
        self.aruco_on.setCheckable(False)
        self.aruco_on.setAutoDefault(False)
        self.aruco_on.setDefault(False)
        self.aruco_on.setFlat(False)
        self.aruco_on.setObjectName("aruco_on")
        self.aruco_off = QtWidgets.QPushButton(self.groupBox)
        self.aruco_off.setGeometry(QtCore.QRect(20, 170, 131, 61))
        self.aruco_off.setObjectName("aruco_off")
        self.backhome = QtWidgets.QPushButton(self.centralwidget)
        self.backhome.setGeometry(QtCore.QRect(280, 1470, 111, 61))
        self.backhome.setObjectName("backhome")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(260, 10, 1331, 1441))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1600, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "AMR-Control"))
        self.groupBox.setTitle(_translate("MainWindow", "ArUCo_Image"))
        self.aruco_on.setText(_translate("MainWindow", "On"))
        self.aruco_off.setText(_translate("MainWindow", "Off"))
        self.backhome.setText(_translate("MainWindow", "back"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
