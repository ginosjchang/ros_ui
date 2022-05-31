# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/node_checker.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_NodeChecker(object):
    def setupUi(self, NodeChecker):
        NodeChecker.setObjectName("NodeChecker")
        NodeChecker.resize(180, 288)
        self.gridLayout = QtWidgets.QGridLayout(NodeChecker)
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QtWidgets.QLabel(NodeChecker)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 1, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(NodeChecker)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 3, 1, 1, 1)
        self.label_9 = QtWidgets.QLabel(NodeChecker)
        self.label_9.setObjectName("label_9")
        self.gridLayout.addWidget(self.label_9, 4, 1, 1, 1)
        self.gripper_status_button = QtWidgets.QPushButton(NodeChecker)
        self.gripper_status_button.setStyleSheet("border: 0px;")
        self.gripper_status_button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("ui/../icon/icons8-close-512.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.gripper_status_button.setIcon(icon)
        self.gripper_status_button.setObjectName("gripper_status_button")
        self.gridLayout.addWidget(self.gripper_status_button, 6, 0, 1, 1)
        self.detect_status_button = QtWidgets.QPushButton(NodeChecker)
        self.detect_status_button.setStyleSheet("border: 0px;")
        self.detect_status_button.setText("")
        self.detect_status_button.setIcon(icon)
        self.detect_status_button.setObjectName("detect_status_button")
        self.gridLayout.addWidget(self.detect_status_button, 2, 0, 1, 1)
        self.label = QtWidgets.QLabel(NodeChecker)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 1, 1, 1)
        self.velodyne_status_button = QtWidgets.QPushButton(NodeChecker)
        self.velodyne_status_button.setStyleSheet("border: 0px;")
        self.velodyne_status_button.setText("")
        self.velodyne_status_button.setIcon(icon)
        self.velodyne_status_button.setObjectName("velodyne_status_button")
        self.gridLayout.addWidget(self.velodyne_status_button, 3, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(NodeChecker)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 2, 1, 1, 1)
        self.label_13 = QtWidgets.QLabel(NodeChecker)
        self.label_13.setObjectName("label_13")
        self.gridLayout.addWidget(self.label_13, 6, 1, 1, 1)
        self.rtab_status_button = QtWidgets.QPushButton(NodeChecker)
        self.rtab_status_button.setStyleSheet("border: 0px;")
        self.rtab_status_button.setText("")
        self.rtab_status_button.setIcon(icon)
        self.rtab_status_button.setObjectName("rtab_status_button")
        self.gridLayout.addWidget(self.rtab_status_button, 4, 0, 1, 1)
        self.rs_status_button = QtWidgets.QPushButton(NodeChecker)
        self.rs_status_button.setEnabled(True)
        self.rs_status_button.setStyleSheet("border: 0px;")
        self.rs_status_button.setText("")
        self.rs_status_button.setIcon(icon)
        self.rs_status_button.setObjectName("rs_status_button")
        self.gridLayout.addWidget(self.rs_status_button, 0, 0, 1, 1)
        self.label_11 = QtWidgets.QLabel(NodeChecker)
        self.label_11.setObjectName("label_11")
        self.gridLayout.addWidget(self.label_11, 5, 1, 1, 1)
        self.yolo_status_button = QtWidgets.QPushButton(NodeChecker)
        self.yolo_status_button.setStyleSheet("border: 0px;")
        self.yolo_status_button.setText("")
        self.yolo_status_button.setIcon(icon)
        self.yolo_status_button.setObjectName("yolo_status_button")
        self.gridLayout.addWidget(self.yolo_status_button, 1, 0, 1, 1)
        self.tm_status_button = QtWidgets.QPushButton(NodeChecker)
        self.tm_status_button.setStyleSheet("border: 0px;")
        self.tm_status_button.setText("")
        self.tm_status_button.setIcon(icon)
        self.tm_status_button.setObjectName("tm_status_button")
        self.gridLayout.addWidget(self.tm_status_button, 5, 0, 1, 1)

        self.retranslateUi(NodeChecker)
        QtCore.QMetaObject.connectSlotsByName(NodeChecker)

    def retranslateUi(self, NodeChecker):
        _translate = QtCore.QCoreApplication.translate
        NodeChecker.setWindowTitle(_translate("NodeChecker", "Checker"))
        self.label_3.setText(_translate("NodeChecker", "Yolo V4   "))
        self.label_7.setText(_translate("NodeChecker", "Velodyne  "))
        self.label_9.setText(_translate("NodeChecker", "RTAB "))
        self.label.setText(_translate("NodeChecker", "RealSense"))
        self.label_5.setText(_translate("NodeChecker", "Detection  "))
        self.label_13.setText(_translate("NodeChecker", "Gripper  "))
        self.label_11.setText(_translate("NodeChecker", "TM_ARM"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    NodeChecker = QtWidgets.QWidget()
    ui = Ui_NodeChecker()
    ui.setupUi(NodeChecker)
    NodeChecker.show()
    sys.exit(app.exec_())
