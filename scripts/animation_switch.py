#! /usr/bin/env python3
"""Animation switch for the actor plugin in Gazebo."""

import os
import yaml
import rospy
import rospkg
from geometry_msgs.msg import Twist, Vector3
from pynput import keyboard
from pynput.keyboard import Key
from gazebo_ros_actor_plugin.msg import ActorAnimation


class AnimationSwitch():
    """Class to teleoperate the animation using arrow keys."""

    def __init__(self):
        """Initialize the AnimationSwitch class."""
        self.update_rate = rospy.get_param('~update_rate', 50)
        self.time_period = 1./self.update_rate

        package_path = rospkg.RosPack().get_path('gazebo_ros_actor_plugin')
        animation_config = os.path.join(package_path, 'config',
                                        'params', 'animation_config.yaml')

        self.animation_switch_config = rospy.get_param('~animation_switch_config', animation_config)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 5.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 5.0)
        self.linear_vel = rospy.get_param('~linear_vel_start', 1.5)
        self.angular_vel = rospy.get_param('~angular_vel_start', 2.5)

        self.set_animation_mapping()

        # Publishers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.animation_pub = \
            rospy.Publisher('/cmd_animation', ActorAnimation, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(self.time_period), self.keyboard_update)

    def set_animation_mapping(self):
        """Set the animation mapping."""
        with open(self.animation_switch_config, 'r', encoding='utf-8') as file:
            self.animation_dict = yaml.safe_load(file)

    def forward(self):
        """Move Forward."""
        self.pub.publish(Twist(linear=Vector3(x=self.linear_vel)))

    def backward(self):
        """Move Backward."""
        self.pub.publish(Twist(linear=Vector3(x=-self.linear_vel)))

    def left(self):
        """Move Left."""
        self.pub.publish(Twist(angular=Vector3(z=self.angular_vel)))

    def right(self):
        """Move Right."""
        self.pub.publish(Twist(angular=Vector3(z=-self.angular_vel)))

    def brutestop(self):
        """Stop the robot."""
        self.pub.publish(Twist())

    def key_press(self, key):
        """Listen for key press."""
        try:
            if key == Key.up:
                self.forward()
            elif key == Key.down:
                self.backward()
            elif key == Key.right:
                self.right()
            elif key == Key.left:
                self.left()
            elif key.char == 'w':
                if self.linear_vel < self.max_linear_vel:
                    self.linear_vel += 0.1
                    self.linear_vel = round(self.linear_vel, 1)
                    rospy.loginfo(f"Linear Velocity: {self.linear_vel}")
                else:
                    rospy.loginfo("Reached Max Linear Velocity")
            elif key.char == 's':
                if self.linear_vel > 0.1:
                    self.linear_vel -= 0.1
                    self.linear_vel = round(self.linear_vel, 1)
                    rospy.loginfo(f"Linear Velocity: {self.linear_vel}")
                else:
                    rospy.loginfo("Reached Minimal Linear Velocity")
            elif key.char == 'd':
                if self.angular_vel < self.max_angular_vel:
                    self.angular_vel += 0.1
                    self.angular_vel = round(self.angular_vel, 1)
                    rospy.loginfo(f"Angular Velocity: {self.angular_vel}")
                else:
                    rospy.loginfo("Reached Max Angular Velocity")
            elif key.char == 'a':
                if self.angular_vel > 0.1:
                    self.angular_vel -= 0.1
                    self.angular_vel = round(self.angular_vel, 1)
                    rospy.loginfo(f"Angular Velocity: {self.angular_vel}")
                else:
                    rospy.loginfo("Reached Minimal Angular Velocity")
            elif key.char in self.animation_dict:
                animation = ActorAnimation()
                animation.idle = "idle"
                animation.action = self.animation_dict[key.char]
                self.animation_pub.publish(animation)
            elif key.char == 'q':
                rospy.signal_shutdown("Done")

        except AttributeError:
            pass
        return False

    def key_release(self, _):
        """Listen for key release."""
        self.brutestop()
        return False

    def keyboard_update(self, _):
        """Keyboard Listener for a press and release event."""
        with keyboard.Listener(on_press=self.key_press) \
                as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) \
                as listener_for_key_release:
            listener_for_key_release.join()


def main():
    """Mimic Main Function to initialize the ROS Node and AnimationSwitch class."""
    # Initialize ROS Node
    rospy.init_node('animation_switch_node')

    rospy.loginfo("\n\
        ↑ ↓ ← → : Arrow keys for movement\n\
        w s : Increase/Decrease linear velocity (0.1 m/s)\n\
        d a : Increase/Decrease angular velocity (0.1 rad/s)\n\
        q: Quit\n")

    # Initialize AnimationSwitch
    AnimationSwitch()
    rospy.spin()
    rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
