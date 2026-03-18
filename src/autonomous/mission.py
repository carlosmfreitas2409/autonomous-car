"""Mission planner: tracks pickup/delivery progress through the route.

CV Integration:
    The navigator should call these methods when the camera detects events:
    - get_decision(dp) at each decision point to know INNER vs OUTER
    - record_sign_visit(sign_number) when CV identifies a sign number
      (returns PICKUP, DELIVERY, or NONE — stop the car on PICKUP/DELIVERY)
    - advance_lap() when the car crosses the start line to begin the next lap
    - is_complete to check if the mission is done
"""

from dataclasses import dataclass, field

from src.autonomous.route_table import (
    Decision,
    LapPlan,
    SignAction,
    compute_route,
    signs_for_path,
)


@dataclass
class Mission:
    pickup_sign: int
    delivery_sign: int
    route: list[LapPlan]
    current_lap: int = 0
    pickup_done: bool = False
    delivery_done: bool = False


class MissionPlanner:
    """Orchestrates the route: which decisions to make and when to stop."""

    def __init__(self, pickup: int, delivery: int):
        route = compute_route(pickup, delivery)
        self._mission = Mission(
            pickup_sign=pickup,
            delivery_sign=delivery,
            route=route,
        )

    @property
    def mission(self) -> Mission:
        return self._mission

    @property
    def current_lap_plan(self) -> LapPlan:
        return self._mission.route[self._mission.current_lap]

    @property
    def current_lap(self) -> int:
        return self._mission.current_lap

    @property
    def total_laps(self) -> int:
        return len(self._mission.route)

    def get_decision(self, decision_point: int) -> Decision:
        """Get the decision for a given decision point on the current lap."""
        plan = self.current_lap_plan
        if decision_point == 1:
            return plan.dp1
        if decision_point == 2:
            if plan.dp2 is None:
                raise ValueError("DP2 not reachable (DP1 was INNER)")
            return plan.dp2
        raise ValueError(f"Invalid decision point: {decision_point}")

    def signs_this_lap(self) -> list[int]:
        """Return ordered list of signs the car will pass on the current lap."""
        plan = self.current_lap_plan
        return signs_for_path(plan.dp1, plan.dp2)

    def record_sign_visit(self, sign_number: int) -> SignAction:
        """Record visiting a sign. Returns the action to perform."""
        m = self._mission
        if sign_number == m.pickup_sign and not m.pickup_done:
            m.pickup_done = True
            return SignAction.PICKUP
        if sign_number == m.delivery_sign and m.pickup_done and not m.delivery_done:
            m.delivery_done = True
            return SignAction.DELIVERY
        return SignAction.NONE

    def advance_lap(self) -> None:
        """Move to the next lap. Raises if already on the last lap."""
        if self._mission.current_lap + 1 >= len(self._mission.route):
            raise IndexError("No more laps in the route")
        self._mission.current_lap += 1

    @property
    def is_complete(self) -> bool:
        return self._mission.pickup_done and self._mission.delivery_done
