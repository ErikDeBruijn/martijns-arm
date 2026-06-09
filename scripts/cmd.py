#!/usr/bin/env python3
"""Stuur v19 commando's naar de arm en lees responses.

Voorbeelden:
    ./cmd.py STATUS                       # one-shot status
    ./cmd.py HELP
    ./cmd.py "TUNE KP 1 5.0"
    ./cmd.py MODE PLAYBACK                # mode wisselen
    ./cmd.py --listen 30                  # alleen luisteren (PB stream zien)
    ./cmd.py --port /dev/cu.usbmodemXXX STATUS

Default: leest alleen respons-regels (< prefix). Voeg --pb toe om ook
de PB streaming output door te geven.
"""

import argparse
import serial
import sys
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument("cmd", nargs="*", help="Commando woorden (zonder >). Bv: STATUS")
    p.add_argument("--port", default="/dev/cu.usbmodem1101")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--listen", type=float, default=2.0,
                   help="Aantal sec luisteren na sturen (default 2)")
    p.add_argument("--pb", action="store_true",
                   help="Toon ook PB streaming regels (default: alleen < responses + andere meldingen)")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: kan {args.port} niet openen: {e}", file=sys.stderr)
        sys.exit(1)

    # Even tijd voor ESP32 USB-CDC reset af te ronden
    time.sleep(0.3)
    ser.reset_input_buffer()

    if args.cmd:
        line = ">" + " ".join(args.cmd) + "\n"
        ser.write(line.encode())
        ser.flush()
        print(f"→ {line.strip()}", file=sys.stderr)

    deadline = time.time() + args.listen
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(1024)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").rstrip()
                if not line:
                    continue
                # filter PB regels tenzij --pb
                if line.startswith("PB ") and not args.pb:
                    continue
                print(line)
        else:
            time.sleep(0.02)

    ser.close()


if __name__ == "__main__":
    main()
