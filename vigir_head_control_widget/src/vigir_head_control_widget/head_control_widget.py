import os
import thread

import rospy
import rospkg
import tf

from math import pi

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PySide import QtCore

from std_msgs.msg import String
from vigir_planning_msgs.msg import HeadControlCommand
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


from PySide.QtGui import QComboBox


class HeadControl(Plugin):

    manualControlsEnabled = False
    running = True

    def __init__(self, context):
        super(HeadControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Head Control')

        # Publisher
        self.modePublisher = rospy.Publisher('/thor_mang/head_control_mode', HeadControlCommand, queue_size=0)

        # Subscriber
        self.jointSubscriber = rospy.Subscriber('/thor_mang/joint_states', JointState, self.updateJointStates)

        # Tf
        self.tfListener = tf.TransformListener()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('vigir_head_control_widget'), 'resource', 'head_control_widget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('HeadControlUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Connect buttons
        self._widget.modeTrackLeftRadioButton.toggled.connect(self.modeTrackLeftRadioButtonToggled)
        self._widget.modeTrackRightRadioButton.toggled.connect(self.modeTrackRightRadioButtonToggled)
        self._widget.modeManualRadioButton.toggled.connect(self.modeManualRadioButtonToggled)
        self._widget.modeTrackFrameRadioButton.toggled.connect(self.modeTrackFrameRadioButtonToggled)

        self._widget.zeroPushButton.pressed.connect(self.zeroPushButtonPressed)

        self._widget.tiltSpinBox.valueChanged.connect(self.manualJointChanged)
        self._widget.panSpinBox.valueChanged.connect(self.manualJointChanged)

        self.connect(self, QtCore.SIGNAL("panChanged"), self._widget.panSlider.setValue)
        self.connect(self, QtCore.SIGNAL("tiltChanged"), self._widget.tiltSlider.setValue)

        self.connect(self, QtCore.SIGNAL("setComboBoxItems"), self._widget.trackFrameCombobox.addItems)
        self.connect(self, QtCore.SIGNAL("clearComboBox"), self._widget.trackFrameCombobox.clear)

        #TODO: I think, TF starts it's own thread here that does not exit properly
        thread.start_new_thread(self.updateTfFrames,())

    '''
    Radio Buttons
    '''
    def modeManualRadioButtonToggled(self, pressed):
        self.setManualControlsEnabled(pressed)

    def modeTrackLeftRadioButtonToggled(self, pressed):
        command = HeadControlCommand(HeadControlCommand.TRACK_LEFT_HAND, [])
        self.modePublisher.publish(command)

    def modeTrackRightRadioButtonToggled(self, pressed):
        command = HeadControlCommand(HeadControlCommand.TRACK_RIGHT_HAND, [])
        self.modePublisher.publish(command)

    def modeTrackFrameRadioButtonToggled(self, pressed):
        pass

    '''
    Spinbox
    '''
    def panSpinBoxEditingFinished(self):
        self._widget.panDial.setValue(self._widget.panSpinBox.value())
        self._widget.panSlider.setValue(self._widget.panSpinBox.value())

    def tiltSpinBoxEditingFinished(self):
        self._widget.tiltSlider.setValue(self._widget.tiltSpinBox.value())


    def zeroPushButtonPressed(self):
        self._widget.modeOffRadioButton.setChecked(True);
        self._widget.panSpinBox.setValue(0);
        self._widget.tiltSpinBox.setValue(0);
        command = HeadControlCommand(HeadControlCommand.USE_PROVIDED_JOINTS, [0.0, 0.0])
        self.modePublisher.publish(command)

    def manualJointChanged(self):
        if(self.manualControlsEnabled):
            pan = float(self._widget.panSpinBox.value()) / 360.0 * 2 * pi
            tilt = float(self._widget.tiltSpinBox.value()) / 360.0 * 2 * pi
            command = HeadControlCommand(HeadControlCommand.USE_PROVIDED_JOINTS, [pan, tilt])
            self.modePublisher.publish(command)
            print "Sending Manual Joints"

    def setManualControlsEnabled(self, enabled):
        self.manualControlsEnabled = enabled
        self._widget.panGroupBox.setEnabled(enabled)
        self._widget.tiltGroupBox.setEnabled(enabled)

    def updateJointStates(self, jointStateMessage):
        if(not(self.manualControlsEnabled)):
            panIndex = jointStateMessage.name.index('head_pan')
            tiltIndex = jointStateMessage.name.index('head_tilt')
            pan = int(jointStateMessage.position[panIndex] * 360.0 / (2.0*pi))
            tilt = int(jointStateMessage.position[tiltIndex] * 360.0 / (2.0*pi))
            self.emit(QtCore.SIGNAL("panChanged"), pan)
            self.emit(QtCore.SIGNAL("tiltChanged"), tilt)

    def updateTfFrames(self):
        currentFrames = []
        while(self.running):
            try:
                rospy.wait_for_message("/tf", TFMessage, 1)
            except(rospy.ROSException),e:
                pass
            newFrames = self.tfListener.getFrameStrings()
            if not newFrames == currentFrames and self.running:
                currentFrames = newFrames
                self.emit(QtCore.SIGNAL("clearComboBox"), currentFrames)
                self.emit(QtCore.SIGNAL("setComboBoxItems"), currentFrames)
                #print currentFrames


    def shutdown_plugin(self):
        self.running = False
        # TODO unregister all publishers here
        self.jointSubscriber.unregister()
        self.modePublisher.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
