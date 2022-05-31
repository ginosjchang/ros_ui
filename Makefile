all: ui/control_ui.py ui/node_checker_ui.py ui/arm_control_ui.py ui/amm_control_ui.py
	python3 control.py

ui/control_ui.py:ui/control.ui
	pyuic5 -x ui/control.ui -o ui/control_ui.py

ui/node_checker_ui.py:ui/node_checker.ui
	pyuic5 -x ui/node_checker.ui -o ui/node_checker_ui.py

ui/arm_control_ui.py:ui/arm_control.ui
	pyuic5 -x ui/arm_control.ui -o ui/arm_control_ui.py

ui/amm_control_ui.py:ui/amm_control.ui
	pyuic5 -x ui/amm_control.ui -o ui/amm_control_ui.py