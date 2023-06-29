import rospy
from std_msgs.msg import Int8MultiArray
from os import system
from tkinter import Tk, Event


class ControlApp(Tk, object):

    def __init__(self) -> None:
        super(ControlApp, self).__init__()
        self.init_node()
        self.config()
        self.init_keyboard()
        self.mainloop()

    def init_node(self) -> None:
        rospy.init_node("teleop_node")
        self.nav_pub = rospy.Publisher(
            "/nav_action/supervised", Int8MultiArray, queue_size=10)

    def init_keyboard(self) -> None:
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)

    def config(self) -> None:
        self.forw = 32
        self.stop = 16
        self.back = 0
        self.speeds_msg = Int8MultiArray()

    def keydown(self, event: Event) -> None:
        if event.keysym == "Up":
            self.speeds_msg.data = [self.forw, self.forw, self.forw,
                               self.forw, self.forw, self.forw]

        elif event.keysym == "Down":
            self.speeds_msg.data = [self.back, self.back, self.back,
                               self.back, self.back, self.back]
            self.nav_pub.publish(self.speeds_msg)
        elif event.keysym == "Left":
            self.speeds_msg.data = [self.forw, self.back, self.forw,
                               self.back, self.forw, self.back]
            self.nav_pub.publish(self.speeds_msg)
        elif event.keysym == "Right":
            self.speeds_msg.data = [self.back, self.forw, self.back,
                               self.forw, self.back, self.forw]
        self.nav_pub.publish(self.speeds_msg)

    def keyup(self, event: Event) -> None:
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            self.speeds_msg.data = [self.stop, self.stop, self.stop,
                               self.stop, self.stop, self.stop]
            self.nav_pub.publish(self.speeds_msg)


if __name__ == "__main__":
    try:
        system('xset r off')
        control = ControlApp()
        system('xset r on')
    except rospy.ROSInterruptException:
        system('xset r on')
