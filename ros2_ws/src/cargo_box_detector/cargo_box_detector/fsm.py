from enum import Enum


class MissionState(str, Enum):
    EXPLORING = "EXPLORING"
    NAVIGATING_TO_BOX = "NAVIGATING_TO_BOX"
    PICKING_UP = "PICKING_UP"
    NAVIGATING_TO_DROPOFF = "NAVIGATING_TO_DROPOFF"
    DROPPING_OFF = "DROPPING_OFF"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    NO_BOX_FOUND = "NO_BOX_FOUND"


class CargoDeliveryFSM:

    def __init__(self):
        self.state = MissionState.EXPLORING
        self.target_detection = None
        self.failure_reason = None
        self.history = []

    def _transition(self, next_state, event):
        previous_state = self.state

        self.state = next_state

        self.history.append({
            "from": previous_state.value,
            "event": event,
            "to": next_state.value
        })

        return self.state.value

    def on_detection(self, result):
        if self.state != MissionState.EXPLORING:
            return self.state.value

        if not isinstance(result, dict):
            raise TypeError("Detection result must be a dictionary.")

        detections = result.get("detections", [])
        detected_count = result.get("detected_count", 0)

        if detected_count <= 0 or not detections:
            return self.state.value

        self.target_detection = detections[0]

        return self._transition(
            MissionState.NAVIGATING_TO_BOX,
            "BOX_DETECTED"
        )

    def on_exploration_finished(self):
        if self.state != MissionState.EXPLORING:
            raise RuntimeError("Invalid exploration transition.")

        return self._transition(
            MissionState.NO_BOX_FOUND,
            "EXPLORATION_EXHAUSTED"
        )

    def on_navigation_succeeded(self):
        if self.state == MissionState.NAVIGATING_TO_BOX:
            return self._transition(
                MissionState.PICKING_UP,
                "NAVIGATION_TO_BOX_SUCCEEDED"
            )

        if self.state == MissionState.NAVIGATING_TO_DROPOFF:
            return self._transition(
                MissionState.DROPPING_OFF,
                "NAVIGATION_TO_DROPOFF_SUCCEEDED"
            )

        raise RuntimeError("Navigation success received in an invalid state.")

    def on_pickup_succeeded(self):
        if self.state != MissionState.PICKING_UP:
            raise RuntimeError("Pickup success received in an invalid state.")

        return self._transition(
            MissionState.NAVIGATING_TO_DROPOFF,
            "PICKUP_SUCCEEDED"
        )

    def on_dropoff_succeeded(self):
        if self.state != MissionState.DROPPING_OFF:
            raise RuntimeError("Dropoff success received in an invalid state.")

        return self._transition(
            MissionState.COMPLETED,
            "DROPOFF_SUCCEEDED"
        )

    def on_operation_failed(self, reason):
        if self.state in (
            MissionState.COMPLETED,
            MissionState.FAILED,
            MissionState.NO_BOX_FOUND
        ):
            raise RuntimeError("Cannot fail a terminal mission.")

        self.failure_reason = str(reason)

        return self._transition(
            MissionState.FAILED,
            "OPERATION_FAILED"
        )
