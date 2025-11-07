"""
COLREGs Core - Quick Start Example

기본적인 조우 상황 분류와 위험도 평가
"""
from colregs_core import (
    EncounterClassifier,
    RiskAssessment,
    heading_speed_to_velocity
)
import numpy as np

def main():
    print("=" * 60)
    print("COLREGs Core - Quick Start")
    print("=" * 60)
    
    # 1. 초기화
    classifier = EncounterClassifier()
    risk_assessor = RiskAssessment()
    
    # 2. Ship 정보 설정
    print("\n[Ship Information]")
    
    # Own Ship
    os_position = np.array([[-90], [0]])
    os_heading = 0      # North
    os_speed = 6       # m/s
    print(f"Own Ship: pos={os_position}, hdg={os_heading}°, spd={os_speed}m/s")
    
    # Target Ship
    ts_position = np.array([[-60], [0]])
    ts_heading = 0    # North
    ts_speed = 4       # m/s
    print(f"Target Ship: pos={ts_position}, hdg={ts_heading}°, spd={ts_speed}m/s")
    
    # 3. Encounter 분류
    print("\n[Encounter Classification]")
    situation = classifier.classify(
        os_position=os_position,
        os_heading=os_heading,
        os_speed=os_speed,
        ts_position=ts_position,
        ts_heading=ts_heading,
        ts_speed=ts_speed
    )
    
    print(f"Encounter Type: {situation.encounter_type.value.upper()}")
    print(f"Relative Bearing: {situation.relative_bearing:.1f}°")
    print(f"Distance: {situation.distance:.0f} m")
    
    # 4. 위험도 평가
    print("\n[Risk Assessment]")
    os_velocity = heading_speed_to_velocity(os_heading, os_speed)
    ts_velocity = heading_speed_to_velocity(ts_heading, ts_speed)
    
    risk = risk_assessor.assess(
        os_position=os_position,
        os_velocity=os_velocity,
        ts_position=ts_position,
        ts_velocity=ts_velocity
    )
    
    print(f"Risk Level: {risk.risk_level.name}")
    print(f"DCPA: {risk.dcpa:.0f} m ({risk.dcpa/1852:.3f} NM)")
    print(f"TCPA: {risk.tcpa:.0f} s ({risk.tcpa/60:.1f} min)")
    
    if abs(risk.bearing_rate) < 0.1:
        print("⚠️  CONSTANT BEARING - 충돌 위험!")
    
    # 5. 권장 조치
    print("\n[Recommended Actions]")
    
    if risk.requires_action:
        print("⚠️  회피 조치 필요!")
        
        # COLREGs 규칙
        colregs_action = classifier.get_action_requirement(
            situation.encounter_type
        )
        print(f"\nCOLREGs: {colregs_action}")
        
        # 전술적 권장사항
        tactical_action = risk_assessor.get_recommended_action(risk)
        print(f"\nTactical: {tactical_action}")
    else:
        print("✓ 정상 항해 유지")
        print("  계속 감시하고 상황 변화 주시")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
