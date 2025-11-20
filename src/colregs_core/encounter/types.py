"""
COLREGs 기반 Encounter situation types 정의
"""
from enum import Enum
from typing import NamedTuple


class EncounterType(Enum):
    """
    COLREGs Rule 13, 14, 15 기반 조우 상황 분류
    """
    HEAD_ON = "head_on"              # Rule 14: 정면 조우
    OVERTAKING = "overtaking"        # Rule 13: 추월
    CROSSING_GIVE_WAY = "crossing_give_way"    # Rule 15: OS가 give-way vessel
    CROSSING_STAND_ON = "crossing_stand_on"    # Rule 15: OS가 stand-on vessel
    SAFE = "safe"                    # 충돌 위험 없음
    UNDEFINED = "undefined"          # 분류 불가


class RiskLevel(Enum):
    """
    충돌 위험도 등급
    """
    SAFE = 0        # 위험 없음
    LOW = 1         # 낮은 위험
    MEDIUM = 2      # 중간 위험
    HIGH = 3        # 높은 위험
    CRITICAL = 4    # 긴급 상황


class EncounterSituation(NamedTuple):
    """
    조우 상황 분석 결과
    """
    encounter_type: EncounterType
    relative_bearing: float  # degrees, [0, 360)
    relative_course: float   # degrees, (-180, 180]
    distance: float          # meters
    aspect_angle: float      # TS의 aspect angle (degrees)


class CollisionRisk(NamedTuple):
    """
    충돌 위험 평가 결과
    """
    dcpa: float              # Distance at CPA (meters)
    tcpa: float              # Time to CPA (seconds)
    risk_level: RiskLevel
    distance: float          # 현재 거리 (meters)
    bearing_rate: float      # 방위각 변화율 (deg/s)
    
    @property
    def is_dangerous(self) -> bool:
        """위험 상황 여부"""
        return self.risk_level.value >= RiskLevel.HIGH.value
    
    @property
    def requires_action(self) -> bool:
        """회피 조치 필요 여부"""
        return self.risk_level.value >= RiskLevel.MEDIUM.value
