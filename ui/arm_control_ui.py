# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/arm_control.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Arm_control(object):
    def setupUi(self, Arm_control):
        Arm_control.setObjectName("Arm_control")
        Arm_control.resize(400, 300)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("ui/../icon/robot-arm.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        Arm_control.setWindowIcon(icon)
        self.gridLayout = QtWidgets.QGridLayout(Arm_control)
        self.gridLayout.setObjectName("gridLayout")
        self.groupBox_7 = QtWidgets.QGroupBox(Arm_control)
        self.groupBox_7.setObjectName("groupBox_7")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_7)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_5 = QtWidgets.QLabel(self.groupBox_7)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 3, 0, 1, 1)
        self.arm_v_LineEdit = QtWidgets.QLineEdit(self.groupBox_7)
        self.arm_v_LineEdit.setReadOnly(True)
        self.arm_v_LineEdit.setObjectName("arm_v_LineEdit")
        self.gridLayout_2.addWidget(self.arm_v_LineEdit, 4, 1, 1, 1)
        self.arm_w_LineEdit = QtWidgets.QLineEdit(self.groupBox_7)
        self.arm_w_LineEdit.setReadOnly(True)
        self.arm_w_LineEdit.setObjectName("arm_w_LineEdit")
        self.gridLayout_2.addWidget(self.arm_w_LineEdit, 5, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox_7)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 4, 0, 1, 1)
        self.arm_u_LineEdit = QtWidgets.QLineEdit(self.groupBox_7)
        self.arm_u_LineEdit.setReadOnly(True)
        self.arm_u_LineEdit.setObjectName("arm_u_LineEdit")
        self.gridLayout_2.addWidget(self.arm_u_LineEdit, 3, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.groupBox_7)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 5, 0, 1, 1)
        self.arm_y_LineEdit = QtWidgets.QLineEdit(self.groupBox_7)
        self.arm_y_LineEdit.setReadOnly(True)
        self.arm_y_LineEdit.setObjectName("arm_y_LineEdit")
        self.gridLayout_2.addWidget(self.arm_y_LineEdit, 1, 1, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.groupBox_7)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 1, 0, 1, 1)
        self.arm_z_LineEdit = QtWidgets.QLineEdit(self.groupBox_7)
        self.arm_z_LineEdit.setReadOnly(True)
        self.arm_z_LineEdit.setObjectName("arm_z_LineEdit")
        self.gridLayout_2.addWidget(self.arm_z_LineEdit, 2, 1, 1, 1)
        self.arm_x_LineEdit = QtWidgets.QLineEdit(self.groupBox_7)
        self.arm_x_LineEdit.setReadOnly(True)
        self.arm_x_LineEdit.setObjectName("arm_x_LineEdit")
        self.gridLayout_2.addWidget(self.arm_x_LineEdit, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_7)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 0, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.groupBox_7)
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox_7, 0, 0, 1, 1)
        self.groupBox_8 = QtWidgets.QGroupBox(Arm_control)
        self.groupBox_8.setObjectName("groupBox_8")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox_8)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_17 = QtWidgets.QLabel(self.groupBox_8)
        self.label_17.setObjectName("label_17")
        self.gridLayout_3.addWidget(self.label_17, 1, 0, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.groupBox_8)
        self.label_16.setObjectName("label_16")
        self.gridLayout_3.addWidget(self.label_16, 2, 0, 1, 1)
        self.arm_setX_LineEdit = QtWidgets.QLineEdit(self.groupBox_8)
        self.arm_setX_LineEdit.setReadOnly(False)
        self.arm_setX_LineEdit.setObjectName("arm_setX_LineEdit")
        self.gridLayout_3.addWidget(self.arm_setX_LineEdit, 0, 1, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.groupBox_8)
        self.label_12.setObjectName("label_12")
        self.gridLayout_3.addWidget(self.label_12, 4, 0, 1, 1)
        self.arm_setW_LineEdit = QtWidgets.QLineEdit(self.groupBox_8)
        self.arm_setW_LineEdit.setReadOnly(False)
        self.arm_setW_LineEdit.setObjectName("arm_setW_LineEdit")
        self.gridLayout_3.addWidget(self.arm_setW_LineEdit, 5, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.groupBox_8)
        self.label_10.setObjectName("label_10")
        self.gridLayout_3.addWidget(self.label_10, 5, 0, 1, 1)
        self.arm_setZ_LineEdit = QtWidgets.QLineEdit(self.groupBox_8)
        self.arm_setZ_LineEdit.setReadOnly(False)
        self.arm_setZ_LineEdit.setObjectName("arm_setZ_LineEdit")
        self.gridLayout_3.addWidget(self.arm_setZ_LineEdit, 2, 1, 1, 1)
        self.arm_setV_LineEdit = QtWidgets.QLineEdit(self.groupBox_8)
        self.arm_setV_LineEdit.setReadOnly(False)
        self.arm_setV_LineEdit.setObjectName("arm_setV_LineEdit")
        self.gridLayout_3.addWidget(self.arm_setV_LineEdit, 4, 1, 1, 1)
        self.arm_setU_LineEdit = QtWidgets.QLineEdit(self.groupBox_8)
        self.arm_setU_LineEdit.setReadOnly(False)
        self.arm_setU_LineEdit.setObjectName("arm_setU_LineEdit")
        self.gridLayout_3.addWidget(self.arm_setU_LineEdit, 3, 1, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.groupBox_8)
        self.label_13.setObjectName("label_13")
        self.gridLayout_3.addWidget(self.label_13, 3, 0, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.groupBox_8)
        self.label_15.setObjectName("label_15")
        self.gridLayout_3.addWidget(self.label_15, 0, 0, 1, 1)
        self.arm_setY_LineEdit = QtWidgets.QLineEdit(self.groupBox_8)
        self.arm_setY_LineEdit.setReadOnly(False)
        self.arm_setY_LineEdit.setObjectName("arm_setY_LineEdit")
        self.gridLayout_3.addWidget(self.arm_setY_LineEdit, 1, 1, 1, 1)
        self.gridLayout.addWidget(self.groupBox_8, 0, 1, 1, 1)
        self.arm_set_button = QtWidgets.QPushButton(Arm_control)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("ui/../icon/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.arm_set_button.setIcon(icon1)
        self.arm_set_button.setObjectName("arm_set_button")
        self.gridLayout.addWidget(self.arm_set_button, 1, 1, 1, 1)
        self.arm_stop_button = QtWidgets.QPushButton(Arm_control)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("ui/../icon/stop-button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.arm_stop_button.setIcon(icon2)
        self.arm_stop_button.setObjectName("arm_stop_button")
        self.gridLayout.addWidget(self.arm_stop_button, 1, 0, 1, 1)

        self.retranslateUi(Arm_control)
        QtCore.QMetaObject.connectSlotsByName(Arm_control)

    def retranslateUi(self, Arm_control):
        _translate = QtCore.QCoreApplication.translate
        Arm_control.setWindowTitle(_translate("Arm_control", "TM ARM"))
        self.groupBox_7.setTitle(_translate("Arm_control", "Current Pos"))
        self.label_5.setText(_translate("Arm_control", "U"))
        self.arm_v_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.arm_w_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_7.setText(_translate("Arm_control", "V"))
        self.arm_u_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_8.setText(_translate("Arm_control", "W"))
        self.arm_y_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_3.setText(_translate("Arm_control", "Y"))
        self.arm_z_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.arm_x_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_2.setText(_translate("Arm_control", "X"))
        self.label_4.setText(_translate("Arm_control", "Z"))
        self.groupBox_8.setTitle(_translate("Arm_control", "Increment"))
        self.label_17.setText(_translate("Arm_control", "Y"))
        self.label_16.setText(_translate("Arm_control", "Z"))
        self.arm_setX_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_12.setText(_translate("Arm_control", "V"))
        self.arm_setW_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_10.setText(_translate("Arm_control", "W"))
        self.arm_setZ_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.arm_setV_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.arm_setU_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.label_13.setText(_translate("Arm_control", "U"))
        self.label_15.setText(_translate("Arm_control", "X"))
        self.arm_setY_LineEdit.setText(_translate("Arm_control", "0.0"))
        self.arm_set_button.setText(_translate("Arm_control", "Set"))
        self.arm_stop_button.setText(_translate("Arm_control", "Stop"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Arm_control = QtWidgets.QWidget()
    ui = Ui_Arm_control()
    ui.setupUi(Arm_control)
    Arm_control.show()
    sys.exit(app.exec_())
