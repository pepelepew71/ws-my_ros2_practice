#!/usr/bin/env python3

import threading
import tkinter as tk
from tkinter import scrolledtext as ScrolledText

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tk_chatroom.srv import Message

NODE = None
GUI = None


class MyNode(Node):

    def __init__(self, username):
        super().__init__(node_name=username)
        self._username = username
        self.srv_client = self.create_client(srv_type=Message, srv_name="/server/send_message")
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service /server/send_message")
        self.create_subscription(msg_type=String, topic="/chatroom", callback=self.callback_chatroom, qos_profile=10)

    def call_srv_client(self, message: str):
        request = Message.Request()
        request.name = self._username
        request.message = message
        self.srv_client.call_async(request=request)

    def callback_chatroom(self, msg: String):
        try:
            GUI.board.config(state='normal')
            GUI.board.insert(tk.END, msg.data + "\n")
            GUI.board.config(state='disabled')
            GUI.board.see(tk.END)
        except Exception as err:
            print(f'tk_chatroom.client.MyNode error {err}')


class MainFrame(tk.Frame):

    def __init__(self, root=None, username: str= "Anonymous"):
        tk.Frame.__init__(self, master=root)
        self.pack(side='top', fill='both', padx=5, pady=2, expand=False)
        self._username = username
        self._input = None
        self.board = None
        self._setup_gui()

    def _setup_gui(self):
        self._setup_board()
        self._setup_button()
        self._setup_input()
        self._setup_bind()

    def _setup_board(self):
        lab = tk.Label(self, text='Board', anchor='w')
        lab.pack(side='top', pady=2, fill='both', expand=False)
        self.board = ScrolledText.ScrolledText(self, height=15, state='disabled', wrap=tk.WORD)
        self.board.pack(side='top', fill='both', expand=True)

    def _setup_button(self):
        lab = tk.Label(self, text=self._username + ' Say :', anchor='w')
        lab.pack(side='top', pady=2, fill='both', expand=False)
        btn = tk.Button(self, text='Send', command=self._call_service)
        btn.pack(side='right', fill='both', expand=False)

    def _setup_input(self):
        self._input = ScrolledText.ScrolledText(self, height=4, wrap=tk.WORD)
        self._input.pack(side='left', fill='both', padx=(0, 5), expand=True)
        self._input.focus()

    def _setup_bind(self):
        self._input.bind("<KP_Enter>", self._call_service)
        self._input.bind("<Return>", self._call_service)
        self._input.bind("<Shift-Return>", self._dummy)  # disable <Return> event

    def _call_service(self, event=None):
        """
        Call node's service to send text to server.

        About return "break", refs:
        https://stackoverflow.com/questions/24475907/tkinter-text-widget-pressing-return-key-goes-to-line
        """
        try:
            text = self._input.get("1.0", tk.END)
            self._input.delete("1.0", tk.END)
            texts = text.split("\n")
            texts.pop()
            message = "\n".join(texts)
            if message:
                NODE.call_srv_client(message=message)
            else:
                pass
        except Exception as err:
            print(err)
        finally:
            return "break"  # prevent Tkinter from propagating event to other handlers

    def _dummy(self, event=None):
        pass


if __name__ == "__main__":

    username = input("User's name: ")

    rclpy.init()
    NODE = MyNode(username=username)
    thread_spin = threading.Thread(target=rclpy.spin, args=(NODE, ))
    thread_spin.start()

    root = tk.Tk()
    GUI = MainFrame(root=root, username=username)
    root.geometry("400x400")
    root.title("ROS2 Chat Room")
    root.mainloop()

    NODE.destroy_node()
    rclpy.shutdown()
    thread_spin.join()
