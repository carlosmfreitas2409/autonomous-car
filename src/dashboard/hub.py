"""The one place the control loop and the dashboard server meet.

The loop thread only ever does constant-time things here: swap in the newest
state dict and frame references, read the newest drive stick, and drain a
short list of commands. It never waits on a client. The server thread reads
the same slots and does all the slow work (JSON, JPEG, sockets).

Two rules make the network unable to hurt the car:

- Everything outbound is latest-value: a slow client skips states and frames,
  nothing queues behind it.
- E-stop and mode commands are kept in order and never dropped. Drive sticks
  are not queued at all: only the newest one matters, and its arrival time is
  what the supervisor's lease checks.
"""

from collections import deque
from dataclasses import dataclass
from typing import Optional
import itertools
import logging
import threading
import time

import numpy as np

from dashboard.contract import Command
from supervisor import DriveInput

_EVENT_BACKLOG = 200
_MAX_COMMANDS = 64


@dataclass
class Frame:
    image: Optional[np.ndarray] = None
    seq: int = 0


class Hub:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._state: Optional[dict] = None
        self._state_seq = 0
        self._frames = {"drive": Frame(), "signs": Frame()}
        self._commands: deque[Command] = deque()
        self._drive: Optional[DriveInput] = None
        self._events: deque[dict] = deque(maxlen=_EVENT_BACKLOG)
        self._event_seq = itertools.count(1)
        self._replies: dict[int, deque] = {}
        self._controller: Optional[int] = None
        self._clients = itertools.count(1)
        self._connected: set[int] = set()

    # --- loop side -------------------------------------------------------

    def publish(self, state: dict, drive_frame: Optional[np.ndarray], sign_frame: Optional[np.ndarray]) -> None:
        with self._lock:
            state["control"] = {"controller": self._controller, "clients": len(self._connected)}
            self._state = state
            self._state_seq += 1
            if drive_frame is not None:
                frame = self._frames["drive"]
                frame.image, frame.seq = drive_frame, frame.seq + 1
            if sign_frame is not None:
                frame = self._frames["signs"]
                if frame.image is not sign_frame:
                    frame.image, frame.seq = sign_frame, frame.seq + 1

    def drain_commands(self) -> list[Command]:
        with self._lock:
            commands = list(self._commands)
            self._commands.clear()
        return commands

    def latest_drive(self) -> Optional[DriveInput]:
        with self._lock:
            return self._drive

    def reply(self, client: int, message: dict) -> None:
        with self._lock:
            outbox = self._replies.get(client)
            if outbox is not None:
                outbox.append(message)

    def event(self, level: str, source: str, text: str, t: Optional[float] = None) -> None:
        with self._lock:
            self._events.append({
                "seq": next(self._event_seq), "t": round(self._clock() if t is None else t, 3),
                "level": level, "source": source, "text": text,
            })

    # --- server side -----------------------------------------------------

    def connect(self) -> int:
        with self._lock:
            client = next(self._clients)
            self._connected.add(client)
            self._replies[client] = deque(maxlen=256)
        return client

    def disconnect(self, client: int) -> None:
        with self._lock:
            self._connected.discard(client)
            self._replies.pop(client, None)
            released = self._controller == client
            if released:
                self._controller = None
                self._drive = None
        if released:
            self.event("warning", "dashboard", f"console {client} disconnected, control released")

    def claim(self, client: int, force: bool) -> tuple[bool, str]:
        with self._lock:
            holder = self._controller
            if holder is not None and holder != client and not force:
                return False, f"console {holder} has control"
            self._controller = client
        if holder != client:
            note = f" (taken from console {holder})" if holder is not None else ""
            self.event("info", "dashboard", f"console {client} took control{note}")
        return True, ""

    def release(self, client: int) -> None:
        with self._lock:
            if self._controller != client:
                return
            self._controller = None
            self._drive = None
        self.event("info", "dashboard", f"console {client} released control")

    def is_controller(self, client: int) -> bool:
        with self._lock:
            return self._controller == client

    def submit(self, command: Command) -> bool:
        with self._lock:
            if command.type == "drive":
                self._drive = DriveInput(command.args["steering"], command.args["throttle"], self._clock())
                return True
            if len(self._commands) >= _MAX_COMMANDS:
                # Never drop an e-stop: shed the oldest non-safety command.
                for queued in self._commands:
                    if queued.type not in ("estop", "mode"):
                        self._commands.remove(queued)
                        break
                else:
                    if command.type not in ("estop", "mode"):
                        return False
            self._commands.append(command)
            return True

    def state(self) -> tuple[Optional[dict], int]:
        with self._lock:
            return self._state, self._state_seq

    def frame(self, source: str) -> Frame:
        with self._lock:
            frame = self._frames[source]
            return Frame(frame.image, frame.seq)

    def events_since(self, seq: int) -> list[dict]:
        with self._lock:
            return [e for e in self._events if e["seq"] > seq]

    def take_replies(self, client: int) -> list[dict]:
        with self._lock:
            outbox = self._replies.get(client)
            if not outbox:
                return []
            replies = list(outbox)
            outbox.clear()
            return replies


class EventLogHandler(logging.Handler):
    """Feeds the car's own log lines (mission decisions, path changes, stops,
    mode changes) into the dashboard's timeline: they already narrate every
    decision, so the timeline adds no logic of its own."""

    _SKIP = ("aiohttp", "asyncio")

    def __init__(self, hub: Hub):
        super().__init__(level=logging.INFO)
        self._hub = hub

    def emit(self, record: logging.LogRecord) -> None:
        if record.name.startswith(self._SKIP):
            return
        try:
            self._hub.event(record.levelname.lower(), record.name, record.getMessage())
        except Exception:  # never let the timeline break logging
            self.handleError(record)
