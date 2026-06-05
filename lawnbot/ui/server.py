"""FastAPI + WebSocket operator UI server.

Single websocket at /ws:
  - server→client every 1/push_hz seconds: a JSON state snapshot
  - client→server any time: command messages (teleop, mission, teach, tuning)

Static dashboard served from /static.

The runtime object passed in here is the central hub (built in main.py).
The server only reads its public state / calls its public methods; nothing
about the control loop depends on the UI being connected.
"""
from __future__ import annotations

import asyncio
import json
from dataclasses import asdict
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


def build_app(runtime, push_hz: int = 10) -> FastAPI:
    """Wire FastAPI routes against the runtime hub.

    `runtime` is duck-typed; it must expose:
      runtime.snapshot() -> dict
      runtime.command(name: str, payload: dict) -> dict
    """
    app = FastAPI(title="LawnBot")
    static_dir = Path(__file__).parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(str(static_dir / "index.html"))

    @app.post("/cmd/{name}")
    async def http_cmd(name: str, payload: dict) -> dict:
        return runtime.command(name, payload or {})

    @app.websocket("/ws")
    async def ws(socket: WebSocket) -> None:
        await socket.accept()
        period = 1.0 / push_hz
        try:
            push_task = asyncio.create_task(_push_loop(socket, runtime, period))
            try:
                while True:
                    msg = await socket.receive_text()
                    try:
                        obj = json.loads(msg)
                    except json.JSONDecodeError:
                        continue
                    name = obj.get("cmd")
                    payload = obj.get("payload", {})
                    if name:
                        runtime.command(name, payload)
            finally:
                push_task.cancel()
        except WebSocketDisconnect:
            return

    return app


async def _push_loop(socket: WebSocket, runtime, period: float) -> None:
    while True:
        try:
            snap = runtime.snapshot()
            await socket.send_text(json.dumps(snap, default=_default))
        except Exception:
            return
        await asyncio.sleep(period)


def _default(o):
    if hasattr(o, "value"):  # enums
        return o.value
    if hasattr(o, "__dict__"):
        return o.__dict__
    return str(o)
