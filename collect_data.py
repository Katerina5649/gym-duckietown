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
from PIL import Image
import time
import os
# from experiments.utils import save_img

parser = argparse.ArgumentParser()
parser.add_argument('--env-name', default=None)
parser.add_argument('--save-dir', default=None)
parser.add_argument('--map-name', default='udem1')
parser.add_argument('--distortion', default=False, action='store_true')
parser.add_argument('--draw-curve', action='store_true', help='draw the lane following curve')
parser.add_argument('--draw-bbox', action='store_true', help='draw collision detection bounding boxes')
parser.add_argument('--domain-rand', action='store_true', help='enable domain randomization')
parser.add_argument('--frame-skip', default=1, type=int, help='number of frames to skip')
parser.add_argument('--seed', default=1, type=int, help='seed')
args = parser.parse_args()

if args.env_name and args.env_name.find('Duckietown') != -1:
    env = DuckietownEnv(
        seed = args.seed,
        map_name = args.map_name,
        draw_curve = args.draw_curve,
        draw_bbox = args.draw_bbox,
        domain_rand = args.domain_rand,
        frame_skip = args.frame_skip,
        distortion = args.distortion,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.render()

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print('RESET')
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        env.close()
        sys.exit(0)

# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """

    if not os.path.exists(args.save_dir):
        os.makedirs(args.save_dir)

    if not os.path.exists(os.path.join(args.save_dir, 'cross_line')):
        os.makedirs(os.path.join(args.save_dir, 'cross_line'))

    if not os.path.exists(os.path.join(args.save_dir, 'collis')):
        os.makedirs(os.path.join(args.save_dir, 'collis'))

    if not os.path.exists(os.path.join(args.save_dir, 'right_position')):
        os.makedirs(os.path.join(args.save_dir, 'right_position'))

    # car will ride straight
    action = np.array([0.44, 0.0])

    obs, reward, done, info = env.step(action)

    step = env.unwrapped.step_count
    if step % 30 == 0:
        from PIL import Image
        im = Image.fromarray(obs)
        im.save(os.path.join(args.save_dir, 'right_position', f'{time.time()}.png'))

    if done and step > 1:
        print('done!')
        from PIL import Image
        im = Image.fromarray(obs)
        print(info)
        no_collision, all_drivable = info['Simulator']['msg']
        if not no_collision:
            im.save(os.path.join(args.save_dir, 'collis', f'{time.time()}.png'))
        if not all_drivable:
            im.save(os.path.join(args.save_dir, 'cross_line', f'{time.time()}.png'))
        env.reset()
        env.render()

    env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()