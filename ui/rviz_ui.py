from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from rviz import bindings as rviz

class MyViz( QWidget ):
    def __init__(self, config_path=None):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()

        #you will need a rviz config file. This config file basically has the information about what all from the rviz you want to display on your custom UI.
        try:
            reader.readFile( config, config_path )
            self.frame.load( config )
            self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        except:
            pass

        #some settings for how you want your rviz screen to look like.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        self.setLayout( layout )