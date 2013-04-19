#!/usr/bin/python

import Tkinter
from Tkinter import *
from Tkconstants import *

import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy

from arm_plan_pkg.msg import *


class Visualizer(Tkinter.Frame):
    def __init__(self, real):
        rospy.init_node('DTW_Visualizer')
        Tkinter.Frame.__init__(self, root)
        node_prefix = "test_arm"
        rospy.Subscriber(node_prefix + "_online_dtw/matches", DTWMatch, self.callback)
        self.data_lines = []
        #Draw the GUI
        self.headLabel = Label(self, justify=CENTER, font=("Helvetica", 18))
        self.headLabel["text"] = "DTW Trajectory Matching Status"
        self.headLabel.grid(row=0, columnspan=2)
        self.line1 = Label(self, text="--------------------------", font=("Helvetica", 14))
        self.line1.grid(row=1, columnspan=2)
        self.nameslabel = Label(self, text="Trajectory Name", font=("Helvetica", 12))
        self.nameslabel.grid(row=2, column=0)
        self.costslabel = Label(self, text="Match Costs", font=("Helvetica", 12))
        self.costslabel.grid(row=2, column=1)        

    def DrawNewValues(self, names, costs):
        self.data_lines = []
        line_num = 3
        for element in names:
            new_line = Label(self, text=element, font=("Helvetica", 8))
            new_line.grid(row=line_num, column=0)
            self.data_lines.append(new_line)
            line_num = line_num + 1
        line_num = 3
        for cost in costs:
            cost_color = self.colorize(cost, costs)
            cost_val = str(cost)
            new_line = Label(self, text=cost_val, bg=cost_color, fg="white", font=("Courier", 10))
            new_line.grid(row=line_num, column=1)
            self.data_lines.append(new_line)
            line_num = line_num + 1

    def colorize(self, cost, costs):
        """Convert speed value into color"""
        blue = 0
        green = 0
        red = 0
        if (cost == -1.0):
            red = 255
        elif (cost == 0.0):
            blue = 255
        else:
            max_val = max(costs)
            min_val = min(costs)
            rng = max_val - min_val
            green = int(((max_val - cost) / rng) * 255)
        tk_rgb = "#%02x%02x%02x" % (red, green, blue)
        return tk_rgb

    def callback(self, data):
        self.DrawNewValues(data.names, data.costs)


if __name__ == "__main__":
    root = Tkinter.Tk()
    root.title("DTW Match Visualizer")
    Visualizer(root).pack()
    root.mainloop()
