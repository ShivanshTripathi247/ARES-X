#!/usr/bin/env python3
"""
ARES-X Keyboard Controller
Run this on your LAPTOP to drive the rover from terminal

Install deps on laptop first:
  pip install websockets

Run:
  python3 ares_keyboard.py --ip 172.16.61.XXX
"""

import asyncio
import websockets
import json
import sys
import argparse
import tty
import termios

PI_PORT = 8765

HELP = """
╔══════════════════════════════╗
║     ARES-X Keyboard Control  ║
╠══════════════════════════════╣
║  W / ↑  — Forward            ║
║  S / ↓  — Stop               ║
║  A / ←  — Left               ║
║  D / →  — Right              ║
║  X      — Backward           ║
║  E      — Safety ON          ║
║  R      — Safety OFF         ║
║  Q      — Quit               ║
╚══════════════════════════════╝
Hold key to keep moving — release to stop (W/A/D/X only)
"""

KEY_MAP = {
    'w': 'F', 'W': 'F',
    'x': 'B', 'X': 'B',
    'a': 'L', 'A': 'L',
    'd': 'R', 'D': 'R',
    's': 'S', 'S': 'S',
    'e': 'E', 'E': 'E',
    'r': 'D', # safety off
}

MOVE_CMDS = {'F', 'B', 'L', 'R'}

def get_key():
    """Read single keypress without Enter"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            arrows = {'\x41': 'w', '\x42': 'x', '\x43': 'd', '\x44': 'a'}
            ch = arrows.get(ch3, '')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

async def send_cmd(ws, cmd):
    await ws.send(json.dumps({"cmd": cmd}))
    try:
        resp = await asyncio.wait_for(ws.recv(), timeout=0.3)
        data = json.loads(resp)
        if data.get("type") == "ack":
            print(f"\r  CMD: {cmd} | ESP ack: {data.get('esp','?')}      ", end='', flush=True)
    except asyncio.TimeoutError:
        print(f"\r  CMD: {cmd} | (no ack)      ", end='', flush=True)

async def controller(ip):
    uri = f"ws://{ip}:{PI_PORT}"
    print(f"Connecting to ARES-X bridge at {uri} ...")

    try:
        async with websockets.connect(uri, ping_interval=5) as ws:
            # Wait for welcome
            welcome = json.loads(await ws.recv())
            print(f"Connected! Bridge says: {welcome.get('msg')}")
            print(f"Serial to ESP32: {'OK' if welcome.get('serial') else 'NOT CONNECTED — check wiring'}")
            print(HELP)

            last_cmd = None
            while True:
                key = await asyncio.get_event_loop().run_in_executor(None, get_key)

                if key == 'q' or key == 'Q' or key == '\x03':
                    await send_cmd(ws, 'S')
                    print("\nStopped. Goodbye.")
                    break

                cmd = KEY_MAP.get(key)
                if cmd and cmd != last_cmd:
                    await send_cmd(ws, cmd)
                    last_cmd = cmd

                    # Auto-stop after move commands
                    # (mimics button release — remove if you want hold-to-drive)
                    if cmd in MOVE_CMDS:
                        await asyncio.sleep(0.15)
                        await send_cmd(ws, 'S')
                        last_cmd = 'S'

    except ConnectionRefusedError:
        print(f"\nCould not connect to {uri}")
        print("Make sure ares_bridge.py is running on the Pi.")
    except websockets.exceptions.ConnectionClosed:
        print("\nConnection lost.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', required=True, help='Pi IP address e.g. 172.16.61.204')
    args = parser.parse_args()
    asyncio.run(controller(args.ip))