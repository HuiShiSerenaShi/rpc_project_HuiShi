from qt_gui.plugin import Plugin

from .edo_gui_widget import EdoGuiWidget

class EdoGui(Plugin):
    """
    Subclass of Plugin 
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(EdoGui, self).__init__(context)
        self.setObjectName('eDOGUI')

        self._widget = EdoGuiWidget(context)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO implement saving
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO implement restoring
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # TODO move some of the button functionality to config button if it is "more configy"