"""
COLREGs Rule 13, 14, 15 기반 Encounter Situation 분류
"""
import numpy as np
from typing import Tuple, Optional

from ..encounter.types import EncounterType, EncounterSituation
from ..geometry.bearings import (
    calculate_relative_bearing,
    calculate_aspect_angle,
)
from ..utils.utils import WrapTo180, distance

class EncounterClassifier:
    """
    COLREGs 기반 조우 상황 분류기
    
    References:
        - COLREGs Rule 13: Overtaking
        - COLREGs Rule 14: Head-on Situation
        - COLREGs Rule 15: Crossing Situation
    """
    
    # COLREGs 기반 각도 임계값
    OVERTAKING_START = 112.5      # degrees
    OVERTAKING_END = 247.5        # degrees
    HEAD_ON_TOLERANCE = 6.0       # degrees (±6도)
    BOW_CROSSING_START = 5.0      # degrees
    BOW_CROSSING_END = 112.5      # degrees
    STERN_CROSSING_START = 247.5  # degrees
    STERN_CROSSING_END = 355.0    # degrees
    
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
        """
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
        relative_course = WrapTo180(ts_heading - os_heading)
        
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
            relative_course: 상대 침로 [-180, 180]
            aspect_angle: TS의 aspect angle [0, 360)
            os_speed: OS 속도
            ts_speed: TS 속도
        
        Returns:
            EncounterType
        """
        # Rule 13: Overtaking
        # TS가 OS의 선미 섹터(112.5° - 247.5°)에 있고,
        # OS가 TS보다 빠른 경우
        if self._is_overtaking(relative_bearing, aspect_angle, os_speed, ts_speed):
            return EncounterType.OVERTAKING
        
        # Rule 14: Head-on
        # 정면 조우: 양 선박이 거의 반대 방향으로 진행
        if self._is_head_on(relative_bearing, relative_course):
            return EncounterType.HEAD_ON
        
        # Rule 15: Crossing
        if self._is_crossing_give_way(relative_bearing, relative_course):
            return EncounterType.CROSSING_GIVE_WAY
        
        if self._is_crossing_stand_on(relative_bearing, relative_course):
            return EncounterType.CROSSING_STAND_ON
        
        return EncounterType.UNDEFINED
    
    def _is_overtaking(
        self,
        relative_bearing: float,
        aspect_angle: float,
        os_speed: float,
        ts_speed: float
    ) -> bool:
        """
        Rule 13: Overtaking 판단
        
        조건:
        1. OS가 TS의 선미 섹터(112.5° - 247.5°)에서 접근 (TS 기준, aspect_angle 사용)
        2. OS가 TS보다 빠른 속도
        
        Notes:
            COLREGs Rule 13 원문: "A vessel shall be deemed to be overtaking when 
            coming up with another vessel from a direction more than 22.5 degrees 
            abaft HER beam"
            
            여기서 "HER"는 피추월선(vessel being overtaken, 즉 TS)을 의미.
            따라서 TS 입장에서 OS가 선미 섹터에 있는지 판단해야 함.
            이는 aspect_angle (TS에서 OS를 보는 방위)을 사용하여 확인.
        """
        # TS 입장에서 OS가 선미 섹터에 있는지 (COLREGs 정의에 따름)
        overtaking_start = self.OVERTAKING_START - self.overtaking_tolerance
        overtaking_end = self.OVERTAKING_END + self.overtaking_tolerance
        
        ts_stern_sector = overtaking_start <= aspect_angle <= overtaking_end
        
        if not ts_stern_sector:
            return False
        
        # COLREGs에서 overtaking은 추월하는 선박(OS)이 피항 책임
        # OS가 TS보다 빠른지 확인
        speed_check = os_speed > ts_speed * 0.9
        
        return ts_stern_sector and speed_check
    
    def _is_head_on(
        self,
        relative_bearing: float,
        relative_course: float
    ) -> bool:
        """
        Rule 14: Head-on Situation 판단
        
        조건:
        1. TS가 OS 정선수 방향 (0° ± tolerance)
        2. 상대 침로가 거의 반대 (180° ± tolerance)
        """
        # TS가 정선수 방향
        dead_ahead = (relative_bearing <= self.HEAD_ON_TOLERANCE or 
                      relative_bearing >= 360 - self.HEAD_ON_TOLERANCE)
        
        # 상대 침로가 반대 방향 (약 180도)
        opposite_course = abs(abs(relative_course) - 180) <= self.HEAD_ON_TOLERANCE * 2
        
        return dead_ahead and opposite_course
    
    def _is_crossing_give_way(self, relative_bearing: float, relative_course: float) -> bool:
        """
        Rule 15: Crossing - OS가 Give-way Vessel
        
        조건:
        1. TS가 OS의 우현(5° - 112.5°)에 위치
        2. 상대 침로가 교차하는 경우 (평행 항해 제외)
        
        Notes:
            COLREGs Rule 15: "two power-driven vessels are CROSSING"
            Crossing은 선박들이 서로 교차하는 침로에 있어야 함.
            평행 항해(relative_course ≈ 0° 또는 180°)는 Crossing이 아님!
        """
        # TS가 우현에 있는지
        in_starboard = self.BOW_CROSSING_START < relative_bearing < self.BOW_CROSSING_END
        
        if not in_starboard:
            return False
        
        # 상대 침로가 교차하는지 (평행 항해 제외)
        # relative_course가 ±30° 이상 차이나야 crossing
        # 0° 근처 (같은 방향) 또는 ±180° 근처 (반대 방향)는 평행 항해
        is_parallel = abs(relative_course) < 30 or abs(abs(relative_course) - 180) < 30
        
        # Crossing은 교차하는 침로에 있어야 함
        return in_starboard and not is_parallel
    
    def _is_crossing_stand_on(self, relative_bearing: float, relative_course: float) -> bool:
        """
        Rule 15: Crossing - OS가 Stand-on Vessel
        
        조건:
        1. TS가 OS의 좌현(247.5° - 355°)에 위치
        2. 상대 침로가 교차하는 경우 (평행 항해 제외)
        
        Notes:
            COLREGs Rule 15: "two power-driven vessels are CROSSING"
            Crossing은 선박들이 서로 교차하는 침로에 있어야 함.
            평행 항해(relative_course ≈ 0° 또는 180°)는 Crossing이 아님!
        """
        # TS가 좌현에 있는지
        in_port = self.STERN_CROSSING_START < relative_bearing < self.STERN_CROSSING_END
        
        if not in_port:
            return False
        
        # 상대 침로가 교차하는지 (평행 항해 제외)
        # relative_course가 ±30° 이상 차이나야 crossing
        # 0° 근처 (같은 방향) 또는 ±180° 근처 (반대 방향)는 평행 항해
        is_parallel = abs(relative_course) < 30 or abs(abs(relative_course) - 180) < 30
        
        # Crossing은 교차하는 침로에 있어야 함
        return in_port and not is_parallel
    
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
