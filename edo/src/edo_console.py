#!/usr/bin/env python3

import rospy
import threading

class EdoConsole(object):

    def __init__(self, commands) -> None:
        super(EdoConsole, self).__init__()

        self.commands = commands
        # Add help command
        self.commands['help'] = {  "desc" : "Prints this list.",
                                    "args" : [],
                                    "types" : [],
                                    "callback" : self.help }
        # Start the keyboard handling thread
        th = threading.Thread(target=self.handle_keyboard)
        th.start()

    def help(self):
        # Print list of commands
        print("-" * 90)
        print(" List of commands: ")
        print("-" * 90)
        for c in self.commands:
        # Prints console command with arguments and description
            print("%-30s %-60s" % (c + ' ' + ' '.join(self.commands[c]['args']), self.commands[c]['desc']))
        print("-" * 90)

    def handle_keyboard(self):
        # Print command list
        self.help()
        while True:
            # Get input
            m = input("> ").lower()
            m = m.split(" ")
            cmd = m[0]
            # Check if we know the command
            if cmd not in self.commands:
                rospy.loginfo(f"Unrecognized command: '{cmd}'.")
            else:
                args = self.commands[cmd]['args']
                types = self.commands[cmd]['types']
                callback = self.commands[cmd]['callback']
                # Check if enough arguments were supplied
                if len(m) - 1 != len(args):
                    rospy.loginfo(f"Not enough arguments for command: '{cmd}'.")
                else:
                    cmd_args = m[1:]
                    if len(cmd_args) != 0:
                        typed_cmd_args = []
                        try:
                            # If there are arguments to the command, cast them to the correct type
                            for i in range(len(cmd_args)):
                                typed_cmd_args.append(types[i](cmd_args[i]))
                            # This is an hack to "overload" the methods
                            # A nicer way would be to provide @overload decorated functions that accept a list as argument for each method
                            if len(typed_cmd_args) == 1:
                                callback(typed_cmd_args[0])
                            elif len(typed_cmd_args) == 2:
                                callback(typed_cmd_args[0], typed_cmd_args[1])
                            elif len(typed_cmd_args) == 3:
                                callback(typed_cmd_args[0], typed_cmd_args[1], typed_cmd_args[2])
                            elif len(typed_cmd_args) == 4:
                                callback(typed_cmd_args[0], typed_cmd_args[1], typed_cmd_args[2], typed_cmd_args[3])
                            else:
                                rospy.loginfo(f"Unimplemented command.")
                        except ValueError:
                            rospy.loginfo(f"Wrong type for argument '{args[i]}': expected {types[i].__name__}, got string.")
                    else:
                        # No arguments needed, just call the method
                        callback()