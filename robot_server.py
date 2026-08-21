from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()


class NavigateRequest(BaseModel):
    destination: str


@app.get("/state")
def get_state():
    return {
        "location": "loading_area",
        "battery": 72,
        "navigation": "idle"
    }


@app.post("/navigate")
def navigate(req: NavigateRequest):
    print(f"Navigation request: {req.destination}")

    # TODO:
    # Send the goal to ROS 2 / Nav2 here.

    return {
        "success": True,
        "destination": req.destination,
        "status": "accepted"
    }


@app.post("/stop")
def stop():
    # TODO:
    # Send stop/cancel command to ROS 2.

    return {
        "success": True,
        "status": "stopped"
    }
