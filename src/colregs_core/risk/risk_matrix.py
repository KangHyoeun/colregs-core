"""
종합 충돌 위험 평가 (Collision Risk Assessment Matrix)
"""
import numpy as np
from typing import Tuple, Optional

from ..encounter.types import RiskLevel, CollisionRisk
from ..risk.cpa_tcpa import calculate_cpa_tcpa, is_collision_course
from ..geometry.bearings import (
    calculate_bearing_rate
)
from ..utils.utils import distance


class RiskAssessment:
    """
    종합 충돌 위험 평가
    
    다음 요소를 종합적으로 고려:
    - DCPA (Distance at CPA)
    - TCPA (Time to CPA)
    - 현재 거리
    - 방위각 변화율 (Constant bearing check)
    - 상대 속도
    """
    
    # 위험도 평가 임계값 (meters)
    DCPA_CRITICAL = 200.0    # 0.1 NM
    DCPA_HIGH = 500.0        # 0.27 NM
    DCPA_MEDIUM = 1000.0     # 0.54 NM
    DCPA_LOW = 2000.0        # 1.08 NM
    
    # 시간 임계값 (seconds)
    TCPA_CRITICAL = 300.0    # 5 minutes
    TCPA_HIGH = 600.0        # 10 minutes
    TCPA_MEDIUM = 1200.0     # 20 minutes
    TCPA_LOW = 1800.0        # 30 minutes
    
    # 방위각 변화율 임계값 (deg/s)
    BEARING_RATE_THRESHOLD = 0.1  # < 0.1 deg/s는 constant bearing
    
    def __init__(
        self,
        dcpa_critical: float = DCPA_CRITICAL,
        dcpa_high: float = DCPA_HIGH,
        dcpa_medium: float = DCPA_MEDIUM,
        dcpa_low: float = DCPA_LOW,
        tcpa_critical: float = TCPA_CRITICAL,
        tcpa_high: float = TCPA_HIGH,
        tcpa_medium: float = TCPA_MEDIUM,
        tcpa_low: float = TCPA_LOW
    ):
        """
        Args:
            dcpa_*: DCPA 임계값들 (meters)
            tcpa_*: TCPA 임계값들 (seconds)
        """
        self.dcpa_critical = dcpa_critical
        self.dcpa_high = dcpa_high
        self.dcpa_medium = dcpa_medium
        self.dcpa_low = dcpa_low
        
        self.tcpa_critical = tcpa_critical
        self.tcpa_high = tcpa_high
        self.tcpa_medium = tcpa_medium
        self.tcpa_low = tcpa_low
    
    def assess(
        self,
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        ts_position: Tuple[float, float],
        ts_velocity: Tuple[float, float]
    ) -> CollisionRisk:
        """
        종합 충돌 위험 평가
        
        Args:
            os_position: Own Ship 위치 (x, y)
            os_velocity: Own Ship 속도 벡터 (vx, vy) m/s
            ts_position: Target Ship 위치 (x, y)
            ts_velocity: Target Ship 속도 벡터 (vx, vy) m/s
        
        Returns:
            CollisionRisk 객체
        """
        # CPA/TCPA 계산
        dcpa, tcpa = calculate_cpa_tcpa(
            os_position, os_velocity,
            ts_position, ts_velocity
        )
        
        # 현재 거리
        current_distance = distance(os_position, ts_position)
        
        # 방위각 변화율
        bearing_rate = calculate_bearing_rate(
            os_position, os_velocity,
            ts_position, ts_velocity
        )
        
        # 위험도 결정
        risk_level = self._determine_risk_level(
            dcpa, tcpa, current_distance, bearing_rate
        )
        
        return CollisionRisk(
            dcpa=dcpa,
            tcpa=tcpa,
            risk_level=risk_level,
            distance=current_distance,
            bearing_rate=bearing_rate
        )
    
    def _determine_risk_level(
        self,
        dcpa: float,
        tcpa: float,
        distance: float,
        bearing_rate: float
    ) -> RiskLevel:
        """
        Risk Level 결정
        
        우선순위:
        1. TCPA < 0 (이미 CPA 통과) → 현재 거리 기반 평가
        2. DCPA & TCPA 조합
        3. Constant bearing 체크
        """
        # TCPA가 음수 (이미 CPA 통과)
        if tcpa < 0:
            # 현재 거리로만 평가
            if distance < self.dcpa_critical:
                return RiskLevel.CRITICAL
            elif distance < self.dcpa_high:
                return RiskLevel.HIGH
            elif distance < self.dcpa_medium:
                return RiskLevel.MEDIUM
            elif distance < self.dcpa_low:
                return RiskLevel.LOW
            else:
                return RiskLevel.SAFE
        
        # TCPA가 무한대 (평행 이동)
        if np.isinf(tcpa):
            if distance < self.dcpa_medium:
                return RiskLevel.LOW
            else:
                return RiskLevel.SAFE
        
        # 위험도 매트릭스 평가
        risk_score = self._calculate_risk_score(dcpa, tcpa, bearing_rate)
        
        if risk_score >= 4.0:
            return RiskLevel.CRITICAL
        elif risk_score >= 3.0:
            return RiskLevel.HIGH
        elif risk_score >= 2.0:
            return RiskLevel.MEDIUM
        elif risk_score >= 1.0:
            return RiskLevel.LOW
        else:
            return RiskLevel.SAFE
    
    def _calculate_risk_score(
        self,
        dcpa: float,
        tcpa: float,
        bearing_rate: float
    ) -> float:
        """
        위험도 점수 계산 (0.0 - 5.0)
        
        DCPA와 TCPA의 조합으로 점수 산정
        Constant bearing일 경우 가중치 증가
        """
        # DCPA 점수 (0-3)
        if dcpa < self.dcpa_critical:
            dcpa_score = 3.0
        elif dcpa < self.dcpa_high:
            dcpa_score = 2.5
        elif dcpa < self.dcpa_medium:
            dcpa_score = 2.0
        elif dcpa < self.dcpa_low:
            dcpa_score = 1.0
        else:
            dcpa_score = 0.0
        
        # TCPA 점수 (0-2)
        if tcpa < self.tcpa_critical:
            tcpa_score = 2.0
        elif tcpa < self.tcpa_high:
            tcpa_score = 1.5
        elif tcpa < self.tcpa_medium:
            tcpa_score = 1.0
        elif tcpa < self.tcpa_low:
            tcpa_score = 0.5
        else:
            tcpa_score = 0.0
        
        # Constant bearing 체크
        # 방위각이 거의 변하지 않으면 충돌 위험 증가
        is_constant_bearing = abs(bearing_rate) < self.BEARING_RATE_THRESHOLD
        bearing_multiplier = 1.2 if is_constant_bearing else 1.0
        
        # 최종 점수
        risk_score = (dcpa_score + tcpa_score) * bearing_multiplier
        
        return min(risk_score, 5.0)
    
    def get_recommended_action(self, risk: CollisionRisk) -> str:
        """
        위험도에 따른 권장 조치
        
        Args:
            risk: CollisionRisk 객체
        
        Returns:
            권장 조치 설명
        """
        actions = {
            RiskLevel.SAFE: (
                "정상 항해 유지. 지속적인 감시 필요."
            ),
            RiskLevel.LOW: (
                f"주의 항해. DCPA {risk.dcpa:.0f}m, TCPA {risk.tcpa:.0f}s. "
                "상황 지속 모니터링."
            ),
            RiskLevel.MEDIUM: (
                f"경계 강화. DCPA {risk.dcpa:.0f}m, TCPA {risk.tcpa:.0f}s. "
                "조기 회피 조치 검토. COLREGs 확인."
            ),
            RiskLevel.HIGH: (
                f"즉시 회피 조치 필요. DCPA {risk.dcpa:.0f}m, TCPA {risk.tcpa:.0f}s. "
                "대각도 변침 또는 큰 폭 감속 권장."
            ),
            RiskLevel.CRITICAL: (
                f"긴급 상황! DCPA {risk.dcpa:.0f}m, TCPA {risk.tcpa:.0f}s. "
                "즉각적인 긴급 회피 조치 실행. 필요시 음향신호 및 통신."
            )
        }
        return actions.get(risk.risk_level, "Unknown risk level")
    
    def assess_multiple_targets(
        self,
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        targets: list  # List of (ts_position, ts_velocity) tuples
    ) -> list:
        """
        다중 목표선에 대한 위험 평가
        
        Args:
            os_position: Own Ship 위치
            os_velocity: Own Ship 속도
            targets: [(ts_position, ts_velocity), ...] 리스트
        
        Returns:
            위험도 순으로 정렬된 CollisionRisk 리스트
        """
        risks = []
        for ts_position, ts_velocity in targets:
            risk = self.assess(
                os_position, os_velocity,
                ts_position, ts_velocity
            )
            risks.append(risk)
        
        # 위험도 순 정렬 (CRITICAL 우선)
        risks.sort(key=lambda r: (-r.risk_level.value, r.tcpa))
        
        return risks
    
    def get_most_dangerous_target(
        self,
        os_position: Tuple[float, float],
        os_velocity: Tuple[float, float],
        targets: list
    ) -> Optional[Tuple[int, CollisionRisk]]:
        """
        가장 위험한 목표선 식별
        
        Returns:
            (target_index, CollisionRisk) 또는 None
        """
        risks = self.assess_multiple_targets(
            os_position, os_velocity, targets
        )
        
        if not risks:
            return None
        
        most_dangerous = risks[0]
        if most_dangerous.risk_level == RiskLevel.SAFE:
            return None
        
        # 원래 타겟 리스트에서 인덱스 찾기
        for idx, (ts_pos, ts_vel) in enumerate(targets):
            risk = self.assess(os_position, os_velocity, ts_pos, ts_vel)
            if (risk.dcpa == most_dangerous.dcpa and 
                risk.tcpa == most_dangerous.tcpa):
                return (idx, most_dangerous)
        
        return None
