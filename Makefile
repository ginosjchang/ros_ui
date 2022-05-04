control_ui.py:control.ui
	pyuic5 -x control.ui -o control_ui.py

test: control_ui.py
	python3 control.py

