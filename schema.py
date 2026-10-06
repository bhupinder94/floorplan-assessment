"""
schema.py
Output contract for the capture -> floor plan + damage pipeline.

Every measurement is (value, confidence_m) so intervals can widen honestly
as input data thins out (photos < video < lidar). No pydantic dependency
on purpose -- stdlib dataclasses only, so this runs with zero extra installs.
"""
from dataclasses import dataclass, field, asdict
from typing import List, Tuple, Optional
import json


@dataclass
class Measurement:
    value_m: float
    confidence_m: float   # +/- interval in meters. Wider = less sure. Never 0.
    source: str            # "lidar" | "video" | "photo"


@dataclass
class Opening:
    kind: str               # "door" | "window"
    wall_id: str
    width: Measurement
    position_m: float       # distance along the wall from its start point


@dataclass
class Wall:
    wall_id: str
    start_xy: Tuple[float, float]
    end_xy: Tuple[float, float]
    length: Measurement


@dataclass
class RoomPlan:
    room_id: str
    tier: str               # "lidar" | "video" | "photo"
    walls: List[Wall]
    ceiling_height: Measurement
    floor_area_m2: Measurement
    openings: List[Opening] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)   # e.g. "ceiling not observed in capture"


@dataclass
class DamageRegion:
    room_id: str
    surface: str            # e.g. "wall_0", "ceiling"
    damage_class: str       # "crack" | "stain" | "mold" | ...
    extent_m: float
    confidence: float       # 0-1


@dataclass
class ConcealedDamageFlag:
    room_id: str
    surface: str
    rule_fired: str
    note: str


@dataclass
class Adjacency:
    room_a: str
    room_b: str
    via: str                # opening id connecting them


@dataclass
class PropertyPlan:
    capture_id: str
    tier: str
    rooms: List[RoomPlan]
    adjacency: List[Adjacency] = field(default_factory=list)
    damage: List[DamageRegion] = field(default_factory=list)
    concealed: List[ConcealedDamageFlag] = field(default_factory=list)

    def to_json(self, path: str):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @staticmethod
    def from_single_room(room: RoomPlan, capture_id: str) -> "PropertyPlan":
        return PropertyPlan(capture_id=capture_id, tier=room.tier, rooms=[room])