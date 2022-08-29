#!/usr/bin/env python

import os
import sys
import array
import struct

from fcntl import ioctl, F_SETFL, F_GETFL


class Input(object):
    def __init__(self, index=0):
        # devices
        self.device = None
        self.index = index

        # axis names
        self.axis_names = {
            0x00: 'lx',
            0x01: 'ly',
            0x02: 'l2', #'rx', +
            0x03: 'rx', #'l2', +
            0x04: 'ry', #'r2',
            0x05: 'r2', #'ry',
            0x06: 'throttle',
            0x07: 'rudder',
            0x08: 'wheel',
            0x09: 'gas',
            0x0a: 'brake',
            0x10: 'x',
            0x11: 'y',
            0x12: 'x1',
            0x13: 'y1',
            0x14: 'x2',
            0x15: 'y2',
            0x16: 'x3',
            0x17: 'y3',
            0x18: 'pressure',
            0x19: 'distance',
            0x1a: 'tx',
            0x1b: 'ty',
            0x1c: 'tw',
            0x20: 'volume',
            0x28: 'misc'
        }

        # button names
        self.button_names = {
            0x120: 'trigger',
            0x121: 'thumb1',
            0x122: 'thumb2',
            0x123: 'top1',
            0x124: 'top2',
            0x125: 'pinkie',
            0x126: 'base1',
            0x127: 'base2',
            0x128: 'base3',
            0x129: 'base4',
            0x12a: 'base5',
            0x12b: 'base6',
            0x12f: 'dead',
            0x130: 'a',
            0x131: 'b',
            0x132: 'c',
            0x133: 'd',
            0x134: 'l1',
            0x135: 'r1',
            0x136: 'l2',
            0x137: 'r2',
            0x138: 'lt',
            0x139: 'rt',
            0x13a: 'l',
            0x13b: 'r',
            0x13c: 'home',
            0x13d: 'center',
            0x13e: 'back',
            0x220: 'dpad_up',
            0x221: 'dpad_down',
            0x222: 'dpad_left',
            0x223: 'dpad_right',
            0x2c0: 'dpad_left',
            0x2c1: 'dpad_right',
            0x2c2: 'dpad_up',
            0x2c3: 'dpad_down'
        }

        # mapping names
        self.axis_map = []
        self.button_map = []

        # mapping states
        self.axis_states = {}
        self.button_states = {}

        # connect
        self.connect()

    def connect(self):
        self.device = open(f'/dev/input/js{self.index}', 'rb')
        print(self.device, f' /dev/input/js{self.index}', self.available())
        if self.available():
            return self.map()
        return None

    def map(self):
        # get the device name (JSIOCGNAME)
        buf = array.array('B', [0] * 64)
        ioctl(self.device, 0x80006a13 + (0x10000 * len(buf)), buf)
        device_name = ''.join(map(chr, buf))

        # get number of axes (JSIOCGAXES)
        buf = array.array('B', [0])
        ioctl(self.device, 0x80016a11, buf)
        self.num_axes = buf[0]

        # get number of buttons (JSIOCGBUTTONS)
        buf = array.array('B', [0])
        ioctl(self.device, 0x80016a12, buf)
        self.num_buttons = buf[0]

        # get the axis map (JSIOCGAXMAP)
        buf = array.array('B', [0] * 0x40)
        ioctl(self.device, 0x80406a32, buf)
        for axis in buf[:self.num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # get the button map (JSIOCGBTNMAP)
        buf = array.array('H', [0] * 200)
        ioctl(self.device, 0x80406a34, buf)
        for btn in buf[:self.num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        print('remote[/dev/input/js%d]: connected %s ' % (self.index, device_name))
        print('remote[axes]: %s [%d]' % (', '.join(self.axis_map), self.num_axes))
        print('remote[buttons]: %s [%d]' % (', '.join(self.button_map), self.num_buttons))

        # # now set the input to be nonblocking
        # flags = ioctl(self.devices[self.index], F_GETFL, 0)
        # result = ioctl(self.devices[self.index], F_SETFL, flags | os.O_NONBLOCK)
        #
        result = os.set_blocking(self.device.fileno(), False)
        print(f'result of the ioctl op to set the thing to non-blocking: {result}')
        print(self.button_map)
        print(self.axis_map)

        return True

    def event(self):
        data = self.read()
        if not data:
            return None

        time, value, type, number = struct.unpack('IhBB', data)

        if type & 0x01:
            # print(time, value, type, number)
            button = self.button_map[number]
            # print(button)
            if button:
                self.button_states[button] = value
                return Event(self, 'button', button)

        if type & 0x02:
            axis = self.axis_map[number]
            if axis:
                self.axis_states[axis] = value / 32767.0
                return Event(self, 'axis', axis)

        return Event()

    def read(self):
        if self.available():
            return self.device.read(8)

    def available(self):
        return not self.device is None

    def __del__(self):
        # close all the devices
        try:
            self.device.close()
        except Exception:
            pass



class Event(object):
    def __init__(self, input, target=None, name=None):
        self.input = input
        self.target = target
        self.name = name
        self.handled = None

    def device(self, name):
        if self.input:
            return name == ('dualshock' if self.input.num_buttons else 'absima')
        return None

    def button(self, name):
        active = None
        if self.target == 'button' and self.name == name:
            active = self.input.button_states[name]
            self.handled = self.handled or active
        return active

    def axis(self, name):
        active = None
        if self.target == 'axis' and self.name == name:
            active = self.input.axis_states[name]
            self.handled = self.handled or active
        return active

    def __str__(self):
        return f"({self.input, self.target, self.name, self.handled})"
