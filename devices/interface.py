import time
import numpy as np
from pynput import mouse, keyboard

## Define the thread receiving keyboard for debugging
class Keyboard():

    def __init__(self,
                 pos_sensitivity=6.0,
                 rot_sensitivity=6.0
                ):

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self.single_click_and_hold = False

        self._control = np.zeros(6)
        self._reset_state = 0
        self._enabled = False

        self._rebase = False
        self._grasp = False
        self._reset = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1

        # launch a new listener thread to listen to keyboard
        self.thread = keyboard.Listener(on_press=self._on_press,
                                        on_release=self._on_release)
        self.thread.daemon = True


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset 6-DOF variables
        self.x, self.y = 0, 0
        self.roll, self.pitch = 0, 0
        # Reset control
        self._control = np.zeros(4)
        # Reset grasp
        self.single_click_and_hold = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1


    def _on_press(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)


        if key_char == 'e':
            self._t_last_click = -1
            self._t_click = time.time()
            elapsed_time = self._t_click - self._t_last_click
            self._t_last_click = self._t_click
            self.single_click_and_hold = True

        elif key_char == 'w':
            self.x += 1
        elif key_char == 's':
            self.x -= 1
        elif key_char == 'a':
            self.y += 1
        elif key_char == 'd':
            self.y -= 1
        elif key_char == 'x':
            self.z += 1
        elif key_char == 'z':
            self.y -= 1
        elif key_char == 'o':
            self.roll -= 1
        elif key_char == 'l':
            self.roll += 1
        elif key_char == 'k':
            self.pitch -= 1
        elif key_char == ';':
            self.pitch += 1
        elif key_char == '.':
            self.yaw -= 1
        elif key_char == ',':
            self.yaw += 1
        elif key_char == 'g':
            self._grasp = not self._grasp
        elif key_char == 'r':
            self._rebase = not self._rebase
        elif key_char == 't':
            self._reset = True

        self._control = np.array([self.x,
                                  self.y,
                                  self.z,
                                  self.roll,
                                  self.pitch,
                                  self.yaw])/10.

        print("Key pressed: {}".format(key))

    def _on_release(self, key):

        try:
            key_char = key.char

            if key == keyboard.Key.esc:
                key_char = 'q'

        except AttributeError:
            key_char = str(key)

        if key_char == 'e':
            self.single_click_and_hold = False
        if  key_char == 'q':
            self._reset_state = 1
            self._enabled = False
            self._reset_internal_state()


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0
        # Reset control
        self._control = np.zeros(6)
        # Reset grasp
        self.single_click_and_hold = False


    def start(self):
        self.thread.start()


    @property
    def control(self):
        """
        Grabs current pose of Spacemouse
        Returns:
            np.array: 6-DoF control value
        """
        return self._control

    @property
    def click(self):
        """
        Maps internal states into gripper commands.
        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0


    @property
    def grasp(self):
        return self._grasp


    @property
    def rebase(self):
        return self._rebase


    @property
    def reset(self):
        if self._reset:
            self._reset = False
            return True
        else:
            return False

## Define the thread receiving keyboard for debugging ##
class Mouse():

    def __init__(self,
                 pos_sensitivity=6.0,
                 rot_sensitivity=6.0
                ):

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self.single_click_and_hold = False

        self._control = np.zeros(4)
        self._reset_state = 0
        self._enabled = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1

        # launch a new listener thread to listen to SpaceMouse
        self.thread = mouse.Listener(   on_move=self._on_move,
                                        on_click=self._on_click,
                                        on_scroll=self._on_scroll)
        self.thread.start()

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset 6-DOF variables
        self.x, self.y = 0, 0
        self.roll, self.pitch = 0, 0
        # Reset control
        self._control = np.zeros(4)
        # Reset grasp
        self.single_click_and_hold = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1

    def _on_move(self, x, y):
        if self._flag_init:
            self.x = x - self._x_offset
            self.y = y - self._y_offset
        else:
            self._x_offset = x
            self._y_offset = y
            self._flag_init = True

        self._control[0:2] = np.array([ self.x,
                                        self.y])/100.


    def _on_click(self, x, y, button, pressed):

        self._t_last_click = -1

        self._t_click = time.time()
        elapsed_time = self._t_click - self._t_last_click
        self._t_last_click = self._t_click
        self.single_click_and_hold = True

        # release left button
        if pressed == 0:
            self.single_click_and_hold = False

        # right button is for reset
        if pressed == 1:
            self._reset_state = 1
            self._enabled = False
            self._reset_internal_state()


    def _on_scroll(self, x, y, dx, dy):

        self.roll += dx
        self.pitch += dy

        self.roll = np.clip(self.roll, -1, 1)
        self.pitch = np.clip(self.pitch, -1, 1)

        self._control[2:4] = self.pos_sensitivity * np.array([ self.roll,
                                                              - self.pitch])/10.


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset 6-DOF variables
        self.x, self.y = 0, 0
        self.roll, self.pitch = 0, 0
        # Reset control
        self._control = np.zeros(4)
        # Reset grasp
        self.single_click_and_hold = False


    @property
    def control(self):
        """
        Grabs current pose of Spacemouse
        Returns:
            np.array: 6-DoF control value
        """
        return self._control

    @property
    def click(self):
        """
        Maps internal states into gripper commands.
        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0
