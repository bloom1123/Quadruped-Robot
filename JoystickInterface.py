import pygame
import time
import numpy as np


class JoystickInterface:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def get_info(self):
        joystick_count = pygame.joystick.get_count()
        print('Number of joysticks: {}'.format(joystick_count))

        name = self.joystick.get_name()
        print("Joystick name: {}".format(name))

        axes = self.joystick.get_numaxes()
        print("Number of axes: {}".format(axes))

        buttons = self.joystick.get_numbuttons()
        print("Number of buttons: {}".format(buttons))

    def get_command(self):

        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop

            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

        # todo move velocity_scalar to config
        # x_rate,y_rate, yaw_rate
        velocity_scalar = [-1, -1, -1]

        x_velocity = velocity_scalar[0] * self.joystick.get_axis(1)
        y_velocity = velocity_scalar[1] * self.joystick.get_axis(0)
        z_velocity = 0
        yaw_rate = velocity_scalar[2] * self.joystick.get_axis(2)
        roll_rate = 0.0
        return [x_velocity, y_velocity, yaw_rate, roll_rate]

    def get_buttom(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            # if event.type == pygame.JOYBUTTONDOWN:
            #     print("Joystick button pressed.")
            # if event.type == pygame.JOYBUTTONUP:
                # print("Joystick button released.")
        start_buttom = self.joystick.get_button(7)
        back_bottom = self.joystick.get_button(6)
        a_buttom = self.joystick.get_button(0)
        b_bottom = self.joystick.get_button(1)
        x_buttom = self.joystick.get_button(2)
        y_bottom = self.joystick.get_button(3)

        # if start_button:
        #     print('start')
        # if back_bottom:
        #     print('back')
        # if a_buttom:
        #     print('a')
        # if b_bottom:
        #     print('b')
        # if x_buttom:
        #     print('x')
        # if y_bottom:
        #     print('y')
        return [start_buttom, back_bottom, a_buttom, b_bottom, x_buttom, y_bottom]

if __name__ == '__main__':
    command = JoystickInterface()
    command.get_info()
    while True:
        action = command.get_command()
        print(action)
        time.sleep(0.01)
