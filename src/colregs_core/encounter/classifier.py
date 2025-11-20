"""
COLREGs Rule 13, 14, 15 기반 Encounter Situation 분류
"""
import numpy as np
from typing import Tuple, Optional

from .types import EncounterType, EncounterSituation
from ..geometry import (
    calculate_relative_bearing,
    calculate_aspect_angle,
)
from ..utils import WrapTo360, distance

class EncounterClassifier:
    """
    COLREGs 기반 조우 상황 분류기
    
    References:
        - COLREGs Rule 13: Overtaking
        - COLREGs Rule 14: Head-on Situation
        - COLREGs Rule 15: Crossing Situation
    """
    
    # COLREGs 기반 각도 임계값
    R1_RANGE = 22.5       # degrees (+-22.5도)
    R2_START = 22.5
    R2_END = 90.0
    R3_START = 90.0
    R3_END = 112.5
    R4_START = 112.5
    R4_END = 247.5
    R5_START = 247.5
    R5_END = 270.0
    R6_START = 270.0
    R6_END = 337.5

    TSR1_RANGE = 67.5       # degrees (+-67.5도)
    TSR2_START = 67.5
    TSR2_END = 90.0
    TSR3_START = 90.0
    TSR3_END = 157.5
    TSR4_START = 157.5
    TSR4_END = 202.5
    TSR5_START = 202.5
    TSR5_END = 270.0
    TSR6_START = 270.0
    TSR6_END = 292.5

    def __init__(
        self,
        safe_distance: float = 2000.0,  # meters
        overtaking_tolerance: float = 0.0  # additional tolerance
    ):
        """
        Args:
            safe_distance: 안전 거리 임계값 (meters)
            overtaking_tolerance: 추월 각도 허용오차 (degrees)
        """
        self.safe_distance = safe_distance
        self.overtaking_tolerance = overtaking_tolerance
    
    def classify(
        self,
        os_position: Tuple[float, float],
        os_heading: float,
        os_speed: float,
        ts_position: Tuple[float, float],
        ts_heading: float,
        ts_speed: float
    ) -> EncounterSituation:
        """
        조우 상황 분류
        
        Args:
            os_position: Own Ship 위치 (x, y) meters
            os_heading: Own Ship heading (degrees, 0=North, CW)
            os_speed: Own Ship speed (m/s)
            ts_position: Target Ship 위치 (x, y) meters
            ts_heading: Target Ship heading (degrees)
            ts_speed: Target Ship speed (m/s)
        
        Returns:
            EncounterSituation 객체
        
        Raises:
            ValueError: If positions are invalid
        """
        # Input validation
        if not isinstance(os_position, (tuple, list)) or len(os_position) != 2:
            raise ValueError(f"os_position must be a tuple/list of length 2. Got {os_position}")
        
        if not isinstance(ts_position, (tuple, list)) or len(ts_position) != 2:
            raise ValueError(f"ts_position must be a tuple/list of length 2. Got {ts_position}")
        
        # 거리 계산
        current_distance = distance(os_position, ts_position)
        
        # 안전 거리 밖이면 SAFE
        if current_distance > self.safe_distance:
            return EncounterSituation(
                encounter_type=EncounterType.SAFE,
                relative_bearing=0.0,
                relative_course=0.0,
                distance=current_distance,
                aspect_angle=0.0
            )
        
        # 상대 방위각 계산
        relative_bearing = calculate_relative_bearing(
            os_position, os_heading, ts_position
        )
        
        # 상대 침로 계산
        relative_course = WrapTo360(ts_heading - os_heading)
        
        # Aspect angle 계산 (TS에서 OS를 보는 방위)
        aspect_angle = calculate_aspect_angle(
            ts_heading, os_position, ts_position
        )
        
        # Encounter type 분류
        encounter_type = self._classify_encounter_type(
            relative_bearing, relative_course, aspect_angle,
            os_speed, ts_speed
        )
        
        return EncounterSituation(
            encounter_type=encounter_type,
            relative_bearing=relative_bearing,
            relative_course=relative_course,
            distance=current_distance,
            aspect_angle=aspect_angle
        )
    
    def _classify_encounter_type(
        self,
        relative_bearing: float,
        relative_course: float,
        aspect_angle: float,
        os_speed: float,
        ts_speed: float
    ) -> EncounterType:
        """
        조우 타입 분류 (COLREGs Rule 13, 14, 15)
        
        Args:
            relative_bearing: 상대 방위각 [0, 360)
            relative_course: 상대 침로 [0, 360) - normalized difference between TS and OS headings
            aspect_angle: TS의 aspect angle [0, 360)
            os_speed: OS 속도
            ts_speed: TS 속도
        
        Returns:
            EncounterType
        """
        # Rule 13: Overtaking
        # Aspect angle 기준 R4 + Relative course 기준 TSR1 + 속도 비교
        if self._is_overtaking(relative_bearing, relative_course, aspect_angle, os_speed, ts_speed):
            return EncounterType.OVERTAKING
        
        # Rule 14: Head-on
        # Relative bearing 기준 R 6, 1, 2 중 하나에 있고
        # Relative course 기준 TSR 4에 있는 경우
        if self._is_head_on(relative_bearing, relative_course):
            return EncounterType.HEAD_ON
        
        # Rule 15: Crossing
        # Crossing Give-way
        # Relative bearing 기준 R 1, 2, 3 중 하나에 있고 Relative course 기준 TSR 5, 6 중 하나에 있는 경우
        # Relative bearing 기준 R4에 있고, Relative course 기준 TSR 6에 있는 경우
        if self._is_crossing_give_way(relative_bearing, relative_course):
            return EncounterType.CROSSING_GIVE_WAY

        # Crossing Stand-on
        # Relative bearing 기준 R5, R6, R1 중 하나에 있고 Relative course 기준 TSR 2, 3 중 하나에 있는 경우
        # Relative bearing 기준 R4에 있고, Relative course 기준 TSR 2에 있는 경우
        if self._is_crossing_stand_on(relative_bearing, relative_course):
            return EncounterType.CROSSING_STAND_ON

        # SAFE: 충돌 위험 없음
        return EncounterType.SAFE
    
    def _is_overtaking(
        self,
        relative_bearing: float,
        relative_course: float,
        aspect_angle: float,
        os_speed: float,
        ts_speed: float
    ) -> bool:
        """
        Rule 13: Overtaking 판단
        
        COLREGs Rule 13: "A vessel shall be deemed to be overtaking when coming up 
        with another vessel from a direction more than 22.5 degrees abaft her beam"
        
        조건:
        1. Relative course가 TSR1 영역 (±67.5°) - 거의 같은 방향
        """
        
        # Relative course 기준: 거의 같은 방향으로 항해
        tsr1_sector = (relative_course <= self.TSR1_RANGE or 
                      relative_course >= 360 - self.TSR1_RANGE)
        
        # OS가 TS보다 빠른지 확인
        speed_check = os_speed > ts_speed
        
        return tsr1_sector
    
    def _is_head_on(
        self,
        relative_bearing: float,
        relative_course: float
    ) -> bool:
        """
        Rule 14: Head-on Situation 판단
        """

        r6_sector = self.R6_START < relative_bearing < self.R6_END
        r1_sector = (relative_bearing <= self.R1_RANGE or 
                      relative_bearing >= 360 - self.R1_RANGE)
        r2_sector = self.R2_START < relative_bearing < self.R2_END

        r612_sector = r6_sector or r1_sector or r2_sector
        
        tsr4_sector = self.TSR4_START < relative_course < self.TSR4_END

        return r612_sector and tsr4_sector
    
    def _is_crossing_give_way(self, relative_bearing: float, relative_course: float) -> bool:
        """
        Rule 15: Crossing - OS가 Give-way Vessel
        
        조건:
        1. (R1 or R2 or R3) + (TSR5 or TSR6) - 우현에서 교차
        2. R4 + TSR6 - 후방에서 좌현으로 교차
        """
        r1_sector = (relative_bearing <= self.R1_RANGE or 
                      relative_bearing >= 360 - self.R1_RANGE)
        r2_sector = self.R2_START < relative_bearing < self.R2_END
        r3_sector = self.R3_START < relative_bearing < self.R3_END
        r4_sector = self.R4_START < relative_bearing < self.R4_END

        r123_sector = r1_sector or r2_sector or r3_sector

        tsr5_sector = self.TSR5_START < relative_course < self.TSR5_END
        tsr6_sector = self.TSR6_START < relative_course < self.TSR6_END

        tsr56_sector = tsr5_sector or tsr6_sector
        
        # 조건 1 또는 조건 2를 만족하면 Give-way
        if (r123_sector and tsr56_sector) or (r4_sector and tsr6_sector):
            return True
        
        return False
    
    def _is_crossing_stand_on(self, relative_bearing: float, relative_course: float) -> bool:
        """
        Rule 15: Crossing - OS가 Stand-on Vessel
        
        조건:
        1. (R5 or R6 or R1) + (TSR2 or TSR3) - 좌현에서 교차
        2. R4 + TSR2 - 후방에서 우현으로 교차
        
        Notes:
            COLREGs Rule 15: "two power-driven vessels are CROSSING"
            Crossing은 선박들이 서로 교차하는 침로에 있어야 함.
            평행 항해(relative_course ≈ 0° 또는 180°)는 Crossing이 아님!
        """
        r4_sector = self.R4_START < relative_bearing < self.R4_END
        r5_sector = self.R5_START < relative_bearing < self.R5_END
        r6_sector = self.R6_START < relative_bearing < self.R6_END
        r1_sector = (relative_bearing <= self.R1_RANGE or 
                      relative_bearing >= 360 - self.R1_RANGE)

        r561_sector = r5_sector or r6_sector or r1_sector

        tsr2_sector = self.TSR2_START < relative_course < self.TSR2_END
        tsr3_sector = self.TSR3_START < relative_course < self.TSR3_END

        tsr23_sector = tsr2_sector or tsr3_sector

        # 조건 1 또는 조건 2를 만족하면 Stand-on
        if (r561_sector and tsr23_sector) or (r4_sector and tsr2_sector):
            return True
        
        return False
    
    def get_action_requirement(self, encounter_type: EncounterType) -> str:
        """
        조우 상황별 COLREGs 조치 요구사항
        
        Args:
            encounter_type: 조우 타입
        
        Returns:
            조치 요구사항 설명
        """
        actions = {
            EncounterType.HEAD_ON: (
                "Rule 14: 양 선박 모두 우현으로 변침하여 서로의 좌현을 지나가도록 해야 함"
            ),
            EncounterType.OVERTAKING: (
                "Rule 13: 추월선은 피항선. 피추월선의 진로를 방해하지 않도록 충분히 피항해야 함"
            ),
            EncounterType.CROSSING_GIVE_WAY: (
                "Rule 15: OS가 give-way vessel. 상대선의 진로를 피해야 함. "
                "일반적으로 우현 변침 또는 감속"
            ),
            EncounterType.CROSSING_STAND_ON: (
                "Rule 15: OS가 stand-on vessel. 침로와 속력 유지해야 함. "
                "단, 상대선이 적절한 조치를 하지 않을 경우 Rule 17(a)(ii)에 따라 조치"
            ),
            EncounterType.SAFE: "충돌 위험 없음. 정상 항해 유지",
            EncounterType.UNDEFINED: "상황 분류 불가. 주의 항해 및 상황 관찰"
        }
        return actions.get(encounter_type, "Unknown")
