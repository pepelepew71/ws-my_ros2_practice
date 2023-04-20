#!/usr/bin/env python3

import os
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

PATH = Path(__file__).resolve().parents[0]
ICONS = list()

def set_ICONS():
    """
    Ref:
    https://material.io/resources/icons/?style=baseline
    """
    global ICONS
    ICONS.append(tk.PhotoImage(file=str(PATH / 'icons/outline_add_alarm_black_18dp.png')))
    ICONS.append(tk.PhotoImage(file=str(PATH / 'icons/outline_border_clear_black_18dp.png')))
    ICONS.append(tk.PhotoImage(file=str(PATH / 'icons/outline_lock_black_18dp.png')))
    ICONS.append(tk.PhotoImage(file=str(PATH / 'icons/outline_lock_open_black_18dp.png')))


class MainWindow(tk.Tk, object):

    def __init__(self):
        super(MainWindow, self).__init__()
        set_ICONS()  # tk object must be created after root start

        self.geometry("400x330")
        self.resizable(width=False, height=False)
        self.title("tk interact with ros")

        # self.node = RosNode(parent=self)
        self.frame_kb = LabelFrameKeyboard(master=self)
        self.frame_srv = LabelFrameService(master=self)
        self.frame_action = LabelFrameAction(master=self)

        self._pack_widgets()
        self._setup_bind()

    def _pack_widgets(self):
        self.frame_kb.pack(side='top', fill='x', padx=5, pady=5, expand=False)
        self.frame_srv.pack(side='top', fill='x', padx=5, pady=5, expand=False)
        self.frame_action.pack(side='top', fill='x', padx=5, pady=5, expand=False)

    def _setup_bind(self):
        self.bind('<Escape>', lambda evt: self.destroy())


class LabelFrameKeyboard(tk.LabelFrame, object):

    cmds = ("1", "2", "3", "4")

    def __init__(self, master):
        super(LabelFrameKeyboard, self).__init__(master=master, text="topic /keyboard")

        self.var_lab_str = tk.StringVar()
        self.var_rb = tk.StringVar()
        self.rbs = list()
        self.rbs.append(tk.Radiobutton(master=self, image=ICONS[0], height=36, width=36, indicatoron=0, val=1, variable=self.var_rb, state=tk.DISABLED))
        self.rbs.append(tk.Radiobutton(master=self, image=ICONS[1], height=36, width=36, indicatoron=0, val=2, variable=self.var_rb, state=tk.DISABLED))
        self.rbs.append(tk.Radiobutton(master=self, image=ICONS[2], height=36, width=36, indicatoron=0, val=3, variable=self.var_rb, state=tk.DISABLED))
        self.rbs.append(tk.Radiobutton(master=self, image=ICONS[3], height=36, width=36, indicatoron=0, val=4, variable=self.var_rb, state=tk.DISABLED))
        self.lab_btp = tk.Label(master=self, bitmap='error', relief=tk.RIDGE, height=36, width=36)
        self.lab_str = tk.Label(master=self, textvariable=self.var_lab_str, width=3)

        self._set_vars()
        self._pack_widgets()

    def _set_vars(self):
        self.var_lab_str.set("")
        self.var_rb.set("")

    def _pack_widgets(self):
        self.rbs[0].grid(row=0, column=0)
        self.rbs[1].grid(row=0, column=1)
        self.rbs[2].grid(row=0, column=2)
        self.rbs[3].grid(row=0, column=3)

        self.lab_btp.grid(row=0, column=4, padx=5, pady=2)
        self.lab_str.grid(row=0, column=5, padx=5)

        tk.Label(master=self, text="press key 1, 2, 3, 4 at terminal", anchor="w", padx=5).grid(row=1, column=0, columnspan=6, sticky="w")

    def callback_from_topic(self, txt):
        if txt in self.cmds:
            self._change_rb(txt)
            self._change_lab_btp(txt)
            self._change_lab_str(txt)
        else:
            pass

    def _change_rb(self, cmd):
        self.var_rb.set(cmd)
        for i, rb in enumerate(self.rbs, start=1):
            if i == int(cmd):
                rb.config(state=tk.NORMAL)
            else:
                rb.config(state=tk.DISABLED)

    def _change_lab_btp(self, cmd):
        cmds_map = {"1": "error", "2": "hourglass", "3": "info", "4": "question"}
        self.lab_btp.config(bitmap=cmds_map[cmd])

    def _change_lab_str(self, cmd):
        self.var_lab_str.set(cmd)


class LabelFrameService(tk.LabelFrame, object):

    def __init__(self, master):
        super(LabelFrameService, self).__init__(master=master, text="service /calculate")

        self.var_lab_2 = tk.StringVar()
        self.lab_1 = tk.Label(master=self, text='Expression :')
        self.ent = tk.Entry(master=self, width=15)
        self.btn = tk.Button(master=self, text='=', command=self._calculate)
        self.lab_2 = tk.Label(master=self, textvariable=self.var_lab_2)

        self._set_vars()
        self._pack_widgets()

    def _set_vars(self):
        self.var_lab_2.set("")

    def _pack_widgets(self):
        self.lab_1.pack(side=tk.LEFT)
        self.ent.pack(side=tk.LEFT)
        self.btn.pack(side=tk.LEFT, padx=5, pady=5)
        self.lab_2.pack(side=tk.LEFT)

    def _calculate(self):
        txt = self.ent.get()
        # result = self.master.node.call_service_calculate(txt)
        # self.var_lab_2.set(result)


class LabelFrameAction(tk.LabelFrame, object):

    def __init__(self, master):
        super(LabelFrameAction, self).__init__(master=master, text="action /fibonacci_server")

        self.pro_max_num = None
        self.lab_goal = tk.Label(master=self, text="Fibonacci Goal :")
        self.ent = tk.Entry(master=self, width=10)
        self.btn_goal = tk.Button(master=self, text="Start", command=self._start)
        self.btn_cancel = tk.Button(master=self, text="Cancel", command=self._cancel)
        self.pro = ttk.Progressbar(master=self, orient=tk.HORIZONTAL, length=200, mode="determinate")
        self.text = tk.Text(master=self, height=3, state=tk.DISABLED)

        self._pack_widgets()

    def _pack_widgets(self):
        self.text.pack(side=tk.BOTTOM, fill='x', padx=2, pady=2)
        self.pro.pack(side=tk.BOTTOM, padx=5, pady=5)
        self.lab_goal.pack(side=tk.LEFT)
        self.ent.pack(side=tk.LEFT)
        self.btn_goal.pack(side=tk.LEFT, padx=5, pady=5)
        self.btn_cancel.pack(side=tk.LEFT)

    def _start(self):
        try:
            num = int(self.ent.get())
            # self.master.node.call_action_goal(num=num, feedback_cb=self.feedback, done_cb=self.done)
            self.pro_max_num = num
        except Exception as e:
            print(e)
            # rospy.loginfo("Fibonacci Goal is not an integer")

    def _cancel(self):
        # self.master.node.call_action_cancel()
        pass

    def feedback(self, feedback):
        count = len(feedback.sequence) - 1  # except 0
        progress = float(count) / float(self.pro_max_num) * 100.0
        self.pro["value"] = progress

    def done(self, status, result):
        self.pro["value"] = 0.0
        self.text.config(state=tk.NORMAL)
        self.text.delete(index1='1.0', index2=tk.END)
        self.text.insert(tk.END, str(result.sequence))
        self.text.config(state=tk.DISABLED)


if __name__ == "__main__":

    # try:
    MainWindow()
    tk.mainloop()
    # except rospy.ROSInterruptException as e:
    #     print(e)
