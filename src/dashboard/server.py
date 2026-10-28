from pathlib import Path as FilePath
from typing import Optional
import asyncio
import json
import logging
import threading

import cv2 as cv
from aiohttp import WSMsgType, web

from dashboard.contract import CONTRACT_VERSION, layout, parse_command
from dashboard.hub import Hub

logger = logging.getLogger(__name__)

STATE_PERIOD_S = 0.05
VIDEO_WAIT_S = 1.0
JPEG_QUALITY = 60
_SOURCES = {"drive": 0, "signs": 1}

DIST_DIR = FilePath(__file__).resolve().parents[2] / "web" / "dist"

_NOT_BUILT = """<!doctype html><meta charset=utf-8><title>Cart dashboard</title>
<body style="font:16px system-ui;background:#f4f3ef;color:#1d1c19;padding:40px">
<h1>Dashboard not built</h1><p>The server is running, but <code>web/dist</code> is missing.
Build it once on this machine (no internet is needed afterwards):</p>
<pre>cd web &amp;&amp; bun install &amp;&amp; bun run build</pre></body>"""


class DashboardServer:
    def __init__(self, hub: Hub, host: Optional[str] = None, port: int = 80, dist: FilePath = DIST_DIR):
        self._hub = hub
        self._host = host
        self._port = port
        self._dist = dist
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._runner: Optional[web.AppRunner] = None
        self._ready = threading.Event()
        self._error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name="dashboard-server", daemon=True)

    @property
    def port(self) -> int:
        return self._port

    def start(self, timeout: float = 5.0) -> bool:
        self._thread.start()
        self._ready.wait(timeout)
        if self._error is not None:
            logger.error("dashboard server failed to start: %s", self._error)
            return False
        logger.info("dashboard on http://localhost:%d/", self._port)
        return True

    def stop(self) -> None:
        if self._loop is None:
            return
        future = asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop)
        try:
            future.result(timeout=2.0)
        except Exception:
            pass

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._startup())
        except BaseException as error:
            self._error = error
            self._ready.set()
            return
        self._ready.set()
        self._loop.run_forever()

    async def _startup(self) -> None:
        app = web.Application()
        app.router.add_get("/ws/control", self._control)
        app.router.add_get("/ws/video", self._video)
        app.router.add_get("/", self._index)
        if (self._dist / "assets").is_dir():
            app.router.add_static("/assets", self._dist / "assets")
        app.router.add_get("/{name}", self._root_file)
        self._runner = web.AppRunner(app, access_log=None)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self._host, self._port)
        await site.start()
        if self._port == 0:
            self._port = site._server.sockets[0].getsockname()[1]

    async def _shutdown(self) -> None:
        if self._runner is not None:
            await self._runner.cleanup()
        self._loop.stop()

    async def _index(self, _request: web.Request) -> web.StreamResponse:
        index = self._dist / "index.html"
        if not index.is_file():
            return web.Response(text=_NOT_BUILT, content_type="text/html")
        return web.FileResponse(index, headers={"Cache-Control": "no-cache"})

    async def _root_file(self, request: web.Request) -> web.StreamResponse:
        name = request.match_info["name"]
        candidate = (self._dist / name).resolve()
        if candidate.parent != self._dist.resolve() or not candidate.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(candidate)

    async def _control(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=2.0)
        await ws.prepare(request)
        client = self._hub.connect()
        logger.info("dashboard console %d connected from %s", client, request.remote)
        await ws.send_json({"type": "hello", "version": CONTRACT_VERSION, "layout": layout()})
        pusher = asyncio.ensure_future(self._push(ws, client))
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                await self._handle(ws, client, message.data)
        finally:
            pusher.cancel()
            self._hub.disconnect(client)
            logger.info("dashboard console %d disconnected", client)
        return ws

    async def _handle(self, ws: web.WebSocketResponse, client: int, data: str) -> None:
        try:
            raw = json.loads(data)
        except ValueError:
            await ws.send_json({"type": "nack", "reason": "bad json"})
            return
        command = parse_command(raw)
        command_id = raw.get("id") if isinstance(raw, dict) else None
        if command is None:
            await ws.send_json({"type": "nack", "id": command_id, "cmd": raw.get("type") if isinstance(raw, dict) else None,
                                "reason": "invalid command"})
            return
        command.client = client

        if command.type == "ping":
            await ws.send_json({"type": "pong", "t": command.args["t"]})
            return
        if not self._hub.submit(command):
            await ws.send_json({"type": "nack", "id": command_id, "cmd": command.type, "reason": "busy"})

    async def _push(self, ws: web.WebSocketResponse, client: int) -> None:
        sent_seq = -1
        event_seq = 0
        backlog = self._hub.events_since(0)[-50:]
        if backlog:
            await ws.send_json({"type": "events", "events": backlog})
            event_seq = backlog[-1]["seq"]
        try:
            while not ws.closed:
                state, seq = self._hub.state()
                if state is not None and seq != sent_seq:
                    sent_seq = seq
                    await ws.send_str(json.dumps(state, separators=(",", ":")))
                events = self._hub.events_since(event_seq)
                if events:
                    event_seq = events[-1]["seq"]
                    await ws.send_json({"type": "events", "events": events})
                for reply in self._hub.take_replies(client):
                    await ws.send_json(reply)
                await asyncio.sleep(STATE_PERIOD_S)
        except (ConnectionResetError, asyncio.CancelledError):
            pass

    async def _video(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=2.0, max_msg_size=0)
        await ws.prepare(request)
        source = "drive"
        last_seq = {name: 0 for name in _SOURCES}
        loop = asyncio.get_running_loop()
        async for message in ws:
            if message.type != WSMsgType.TEXT:
                continue
            try:
                request_body = json.loads(message.data)
            except ValueError:
                continue
            if request_body.get("source") in _SOURCES:
                source = request_body["source"]
            if not request_body.get("next"):
                continue
            frame = await self._newer_frame(source, last_seq[source])
            if frame is None or ws.closed:
                await ws.send_bytes(bytes([_SOURCES[source]]))
                continue
            last_seq[source] = frame.seq
            jpeg = await loop.run_in_executor(None, _encode, frame.image)
            if jpeg is None:
                continue
            await ws.send_bytes(bytes([_SOURCES[source]]) + jpeg)
        return ws

    async def _newer_frame(self, source: str, seq: int):
        waited = 0.0
        while waited < VIDEO_WAIT_S:
            frame = self._hub.frame(source)
            if frame.image is not None and frame.seq != seq:
                return frame
            await asyncio.sleep(0.01)
            waited += 0.01
        return None


def _encode(image) -> Optional[bytes]:
    ok, buffer = cv.imencode(".jpg", image, [cv.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return buffer.tobytes() if ok else None
