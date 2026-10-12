from enum import Enum


class NodeType(str, Enum):
    INTERSECTION = "INTERSECTION"
    WAYPOINT = "WAYPOINT"
    WAREHOUSE = "WAREHOUSE"
    SORTING_CENTER = "SORTING_CENTER"
    GATE = "GATE"


class Maneuver(str, Enum):
    LEFT = "LEFT"
    STRAIGHT = "STRAIGHT"
    RIGHT = "RIGHT"
    UTURN = "UTURN"


class ReservationStatus(str, Enum):
    REQUESTED = "REQUESTED"
    GRANTED = "GRANTED"
    COMMITTED = "COMMITTED"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"
    EXPIRED = "EXPIRED"
    MISSED = "MISSED"


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    EN_ROUTE = "EN_ROUTE"
    ARRIVED = "ARRIVED"
    SERVICING = "SERVICING"
    TASK_COMPLETE = "TASK_COMPLETE"
    FAILED = "FAILED"
