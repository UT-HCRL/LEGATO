from pynput import keyboard
import time


class FobButton():

    def __init__(self,
                 time_sensitivity=0.2,
                 mode='toggle',
                ):

        self.time_sensitivity = time_sensitivity

        self._reset_state = 0
        self._enabled = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1
        self._click_and_hold = False
        self._double_click = False
        self._toggle = False

        self._save = False
        self._stop = False
        self._mode = mode

        # launch a new listener thread to listen to keyboard
        self.thread = keyboard.Listener(on_press=self._on_press,
                                        on_release=self._on_release,
                                        )
        self.thread.start()


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """

        # Reset grasp
        self._click_and_hold = False
        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1


    def _on_press(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char == 'a':
            if self._mode != 'toggle':

                self._click_and_hold = True
                self._t_click = time.time()
                elapsed_time = self._t_click - self._t_last_click
                self._t_last_click = self._t_click

                if elapsed_time < self.time_sensitivity:
                    self._double_click = True

        elif key_char == 'q' or key_char == keyboard.Key.esc:
            self._stop = True
            self._reset_state = 0
            self._enabled = False


    def _on_release(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if self._enabled:

            if key_char == "a":
                if self._mode == "toggle":
                    self._toggle = not self._toggle
                else:
                    self._click_and_hold = False
                    self._double_click = False

                if self.click:
                    print("Closing the gripper")
                else:
                    print("Releasing the gripper")

            elif key_char == "s":
                self._reset_state = 0
                self._enabled = False
                self._save = True
                print('Recording successful')

            elif key_char == 'd':
                self._reset_state = 0
                self._enabled = False
                print('Discard recording')

        else:
            if key_char == 'b':
                self._reset_state = 1
                self._enabled = True
                self._reset_internal_state()
                print('Start recording')


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset grasp
        self._click_and_hold = False
        self._double_click = False
        self._toggle = False
        self._save = False


    @property
    def click(self):
        """
        Maps internal states into gripper commands.
        Returns:
            float: Whether we're using single click and hold or not
        """
        if self._mode == 'toggle':
            return self._toggle
        else:
            return self._click_and_hold


    @property
    def enable(self):
        return self._enabled


    @property
    def save(self):
        return self._save


    @property
    def stop(self):
        return self._stop
