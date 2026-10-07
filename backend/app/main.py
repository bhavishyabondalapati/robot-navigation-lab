"""FastAPI server: REST endpoints for maps/planning, a WebSocket for live runs.

Run from the backend/ folder:
    uvicorn app.main:app --reload --port 8000
"""

from __future__ import annotations

import asyncio

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from .grid_map import PRESETS, GridMap, load_preset
from .planners import plan
from .simulation import SimConfig, Simulation

app = FastAPI(title="Robot Navigation Lab")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)

MAX_CELLS = 100 * 100  # keep requests small


class PlanRequest(BaseModel):
    grid: list[list[int]]
    start: tuple[int, int]
    goal: tuple[int, int]
    planner: str = Field("astar", pattern="^(astar|rrt)$")
    seed: int = 0


def parse_grid(rows: list[list[int]]) -> GridMap:
    if not rows or not rows[0] or len(rows) * len(rows[0]) > MAX_CELLS:
        raise ValueError("grid must be non-empty and at most 100x100")
    if any(len(r) != len(rows[0]) for r in rows):
        raise ValueError("grid rows must all have the same length")
    return GridMap.from_list(rows)


@app.get("/api/health")
def health():
    return {"ok": True}


@app.get("/api/presets")
def presets():
    return {"presets": sorted(PRESETS)}


@app.get("/api/maps/{name}")
def get_map(name: str):
    try:
        m, start, goal = load_preset(name)
    except KeyError as e:
        raise HTTPException(404, str(e)) from e
    return {"name": name, "grid": m.to_list(), "start": start, "goal": goal}


@app.post("/api/plan")
def plan_once(req: PlanRequest):
    try:
        grid = parse_grid(req.grid)
    except ValueError as e:
        raise HTTPException(400, str(e)) from e
    return plan(req.planner, grid, req.start, req.goal, seed=req.seed).to_dict()


@app.websocket("/ws/simulate")
async def simulate(ws: WebSocket):
    """Live run protocol (JSON messages).

    client -> server:
      {"type": "start", grid, start, goal, planner, seed, n_particles?}
      {"type": "set_cell", "x", "y", "value": 0|1}   add/remove an obstacle mid-run
      {"type": "pause"} / {"type": "resume"} / {"type": "speed", "delay_ms"}
    server -> client:
      {"type": "plan", ...PlanResult, "replan": bool}  whenever a path is (re)computed
      {"type": "state", ...Simulation.state()}        once per simulation step
      {"type": "error", "message"}
    """
    await ws.accept()
    sim: Simulation | None = None
    paused = False
    delay = 0.05

    async def send_plan(replan: bool):
        await ws.send_json({"type": "plan", "replan": replan, **sim.plan_result.to_dict()})

    try:
        while True:
            # Wait for a message, but only up to `delay` seconds while running:
            # the timeout doubles as the simulation's tick rate.
            running = sim is not None and sim.status == "running" and not paused
            try:
                msg = await asyncio.wait_for(ws.receive_json(), timeout=delay if running else None)
            except asyncio.TimeoutError:
                msg = None

            if msg is not None:
                kind = msg.get("type")
                if kind == "start":
                    try:
                        grid = parse_grid(msg["grid"])
                        cfg = SimConfig(planner=msg.get("planner", "astar"), seed=int(msg.get("seed", 42)),
                                        n_particles=int(min(max(msg.get("n_particles", 400), 50), 2000)))
                        sim = Simulation(grid, tuple(msg["start"]), tuple(msg["goal"]), cfg)
                    except (KeyError, ValueError, TypeError) as e:
                        await ws.send_json({"type": "error", "message": f"bad start message: {e}"})
                        continue
                    paused = False
                    await send_plan(replan=False)
                    await ws.send_json({"type": "state", **sim.state()})
                elif kind == "set_cell" and sim is not None:
                    if sim.set_cell(int(msg["x"]), int(msg["y"]), bool(msg["value"])):
                        await send_plan(replan=True)
                elif kind == "pause":
                    paused = True
                elif kind == "resume":
                    paused = False
                elif kind == "speed":
                    delay = min(max(float(msg.get("delay_ms", 50)), 5), 1000) / 1000
                continue  # handle all queued messages before stepping again

            if running:
                replans_before = sim.replans
                sim.step()
                if sim.replans != replans_before:
                    await send_plan(replan=True)
                await ws.send_json({"type": "state", **sim.state()})
    except WebSocketDisconnect:
        pass
