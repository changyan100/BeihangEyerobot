#!/usr/bin/env python3

import evdev

import asyncio
from evdev import InputDevice, categorize, ecodes

# devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
# for device in devices:
#   print(device.path, device.name, device.phys)

dev = InputDevice('/dev/input/event7')

async def helper(dev):
  async for ev in dev.async_read_loop():
    print(repr(ev))
    

loop = asyncio.get_event_loop()
loop.run_until_complete(helper(dev))