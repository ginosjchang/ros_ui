# ros_ui
## Embeded Rviz
You can install via:
```bash=
sudo apt-get install ros-noetic-visualization-tutorials
```
Import it in your python file via:
```bash=
from rviz import bindings as rviz
```
Use config file data to set rviz config:
```bash=
reader.readFile( config, "my_rviz_file.rviz" )
```
Disable MenuBar with can't open/save config and panels control.
```bash=
self.frame.setMenuBar( None )
```
Disable setStatusBar
```bash=
self.frame.setStatusBar( None )
```