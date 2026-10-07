from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)


def test_presets_and_map():
    names = client.get("/api/presets").json()["presets"]
    assert set(names) == {"maze", "warehouse", "open_field"}
    m = client.get("/api/maps/maze").json()
    assert len(m["grid"]) == 30 and len(m["grid"][0]) == 40


def test_unknown_map_404():
    assert client.get("/api/maps/nope").status_code == 404


def test_plan_endpoint():
    m = client.get("/api/maps/open_field").json()
    for planner in ("astar", "rrt"):
        r = client.post("/api/plan", json={"grid": m["grid"], "start": m["start"], "goal": m["goal"],
                                           "planner": planner, "seed": 1}).json()
        assert r["success"] and r["path_length"] > 0 and r["nodes_expanded"] > 0


def test_websocket_run_streams_states_and_replans():
    m = client.get("/api/maps/open_field").json()
    with client.websocket_connect("/ws/simulate") as ws:
        ws.send_json({"type": "speed", "delay_ms": 5})
        ws.send_json({"type": "start", **m, "planner": "astar", "seed": 0})
        first = ws.receive_json()
        assert first["type"] == "plan" and first["success"]
        path = first["path"]
        states = [ws.receive_json() for _ in range(5)]
        assert all(s["type"] == "state" for s in states)
        assert states[-1]["step"] > states[0]["step"]
        # Block the path well ahead of the robot -> server should send a new plan.
        x, y = path[15]
        ws.send_json({"type": "set_cell", "x": int(x), "y": int(y), "value": 1})
        for _ in range(50):
            msg = ws.receive_json()
            if msg["type"] == "plan":
                assert msg["replan"] and msg["success"]
                break
        else:
            raise AssertionError("no replan message received")
