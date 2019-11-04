#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""

import sys
import argparse
import pyglet
from pyglet.window import key
import numpy as np
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.wrappers import UndistortWrapper
import threading
from PIL import Image

sys.path.append('../Car Interface Weeks 2-3')
import controller


class Car1():

    def __init__(self):
        self.interface = controller.Car_Interface()

        self.pedal_type = None
        self.amount = 0.0
        self.TIME_UNIT = self.interface.dt


    def start(self):

        threading.Timer(self.TIME_UNIT, self.update_pos).start()

    def update_pos(self):

        if (self.interface.gear is not None):
            self.interface.apply_control(self.pedal_type, self.amount)

        threading.Timer(self.TIME_UNIT, self.update_pos).start()

    def gear(self, g):
        if g == "forward":
            Car1.interface.set_gear(Car1.interface.FORWARD)
        elif g == "reverse":
            Car1.interface.set_gear(Car1.interface.REVERSE)

    def pedal(self, p):
        if (p == "accelerate"):
            Car1.pedal_type = Car1.interface.ACCELERATOR
            Car1.amount = 1.0
        elif (p == "brake"):
            Car1.pedal_type = Car1.interface.BRAKE
            Car1.amount = 1.0
        elif (p == "release"):
            Car1.pedal_type = None
            Car1.amount = 0.0

    def turn(self, t):
        if (t == "left"):
            Car1.interface.steer_to(1.0)
        elif (t == "right"):
            Car1.interface.steer_to(-1.0)
        elif (t == "release"):
            Car1.interface.steer_to(0.0)

    def reset(self):
        self.__init__()

    def duckietown_control(self):

        return [self.interface.velocity, self.interface.steering_angle]


class Car2():

    def __init__(self):
        self.interface = controller.Car_Interface()

        self.pedal_type = None
        self.amount = 0.0
        self.TIME_UNIT = self.interface.dt


    def start(self):

        threading.Timer(self.TIME_UNIT, self.update_pos).start()

    def update_pos(self):

        if (self.interface.gear is not None):
            self.interface.apply_control(self.pedal_type, self.amount)

        threading.Timer(self.TIME_UNIT, self.update_pos).start()

    def gear(self, g):
        if g == "forward":
            Car2.interface.set_gear(Car2.interface.FORWARD)
        elif g == "reverse":
            Car2.interface.set_gear(Car2.interface.REVERSE)

    def pedal(self, p):
        if (p == "accelerate"):
            Car2.pedal_type = Car2.interface.ACCELERATOR
            Car2.amount = 1.0
        elif (p == "brake"):
            Car2.pedal_type = Car2.interface.BRAKE
            Car2.amount = 1.0
        elif (p == "release"):
            Car2.pedal_type = None
            Car2.amount = 0.0

    def turn(self, t):
        if (t == "left"):
            Car2.interface.steer_to(1.0)
        elif (t == "right"):
            Car2.interface.steer_to(-1.0)
        elif (t == "release"):
            Car2.interface.steer_to(0.0)

    def reset(self):
        self.__init__()

    def duckietown_control(self):

        return [self.interface.velocity, self.interface.steering_angle]


Car1 = Car1()
Car1.start()


Car2 = Car2()
Car2.start()

# from experiments.utils import save_img

parser = argparse.ArgumentParser()
parser.add_argument('--map-name', default='udem1')
parser.add_argument('--distortion', default=False, action='store_true')
parser.add_argument('--draw-curve', action='store_true', help='draw the lane following curve')
parser.add_argument('--draw-bbox', action='store_true', help='draw collision detection bounding boxes')
parser.add_argument('--domain-rand', action='store_true', help='enable domain randomization')
parser.add_argument('--frame-skip', default=1, type=int, help='number of frames to skip')
parser.add_argument('--seed', default=1, type=int, help='seed')
args = parser.parse_args()

env1 = DuckietownEnv(
    seed = args.seed,
    map_name = args.map_name,
    draw_curve = args.draw_curve,
    draw_bbox = args.draw_bbox,
    domain_rand = args.domain_rand,
    frame_skip = args.frame_skip,
    distortion = args.distortion,
)

env1.do_color_relabeling = False

env1.reset()
env1.render()

env2 = DuckietownEnv(
    seed = args.seed,
    map_name = args.map_name,
    draw_curve = args.draw_curve,
    draw_bbox = args.draw_bbox,
    domain_rand = args.domain_rand,
    frame_skip = args.frame_skip,
    distortion = args.distortion,
)

env2.do_color_relabeling = True

env2.reset()
env2.render()

@env1.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print('RESET')
        env1.reset()
        env1.render()

    elif symbol == key.PAGEUP:
        env1.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        env1.close()
        sys.exit(0)

    # Take a screenshot
    # UNCOMMENT IF NEEDED - Skimage dependency
    # elif symbol == key.RETURN:
    #     print('saving screenshot')
    #     img = env.render('rgb_array')
    #     save_img('screenshot.png', img)


@env2.unwrapped.window.event
def on_key_pressdv(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print('RESET')
        env2.reset()
        env2.render()

    elif symbol == key.PAGEUP:
        env2.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        env2.close()
        sys.exit(0)

    # Take a screenshot
    # UNCOMMENT IF NEEDED - Skimage dependency
    # elif symbol == key.RETURN:
    #     print('saving screenshot')
    #     img = env.render('rgb_array')
    #     save_img('screenshot.png', img)



# Register a keyboard handler
key_handler = key.KeyStateHandler()

env1.unwrapped.window.push_handlers(key_handler)
env2.unwrapped.window.push_handlers(key_handler)

i = 0

def update(dt):
    global i
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """

    if key_handler[key.F]:
        Car1.gear("forward")
        Car2.gear("forward")
    elif key_handler[key.R]:
        Car1.gear("reverse")
        Car2.gear("reverse")
    if key_handler[key.UP]:
        Car1.pedal("accelerate")
        Car2.pedal("accelerate")
    elif key_handler[key.DOWN]:
        Car1.pedal("brake")
        Car2.pedal("brake")
    else:
        Car1.pedal("release")
        Car2.pedal("release")

    if key_handler[key.LEFT]:
        Car1.turn("left")
        Car2.turn("left")
    elif key_handler[key.RIGHT]:
        Car1.turn("right")
        Car2.turn("right")

    else:
        Car1.turn("release")
        Car2.turn("release")

    action = np.array(Car1.duckietown_control())
    action = np.array(Car2.duckietown_control())

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5

    obs1, reward1, done1, info1 = env1.step(action)
    obs2, reward2, done2, info2 = env2.step(action)
    #print('step_count = %s, reward=%.3f' % (env.unwrapped.step_count, reward))

    
    if key_handler[key.RETURN]:
        im = Image.fromarray(obs1)

        im.save('screen.png')

    if done1 or done2:
        print('done!')
        env1.reset()
        env2.reset()
        Car1.reset()
        env1.render()
        env2.render()


    env1.render()
    env2.render()

    # Save image
    from PIL import Image
    im1 = Image.fromarray(obs1)
    im2 = Image.fromarray(obs2)

    im1.save("img/car1/_" + str(i) + ".jpg")
    im2.save("img/car2/" + str(i) + ".jpg")
    i = i + 1


pyglet.clock.schedule_interval(update, 1.0 / env1.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env1.close()
env2.close()


