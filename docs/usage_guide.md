# COLREGs Core - 사용 가이드

## 설치

```bash
cd colregs-core
pip install -e .
```

## 기본 사용법

### 1. Encounter Situation 분류

```python
from colregs_core import EncounterClassifier

# 분류기 초기화
classifier = EncounterClassifier(
    safe_distance=2000.0  # meters, 약 1 NM
)

# 조우 상황 분류
situation = classifier.classify(
    os_position=(0, 0),      # Own Ship 위치 (x, y) meters
    os_heading=0,            # heading (degrees, 0=North, clockwise)
    os_speed=10,             # speed (m/s)
    ts_position=(1000, 500), # Target Ship 위치
    ts_heading=270,          # TS heading
    ts_speed=12              # TS speed
)

print(f"Encounter Type: {situation.encounter_type.value}")
print(f"Relative Bearing: {situation.relative_bearing:.1f}°")
print(f"Distance: {situation.distance:.0f} m")

# COLREGs 조치 확인
action = classifier.get_action_requirement(situation.encounter_type)
print(f"Required Action: {action}")
```

**출력 예시**:
```
Encounter Type: crossing_give_way
Relative Bearing: 26.6°
Distance: 1118 m
Required Action: Rule 15: OS가 give-way vessel. 상대선의 진로를 피해야 함...
```

---

### 2. 충돌 위험 평가

```python
from colregs_core import RiskAssessment, heading_to_velocity

# 위험 평가기 초기화
risk_assessor = RiskAssessment(
    dcpa_critical=200.0,   # DCPA 임계값 (meters)
    tcpa_critical=300.0    # TCPA 임계값 (seconds)
)

# 속도 벡터 계산
os_velocity = heading_to_velocity(heading=0, speed=10)
ts_velocity = heading_to_velocity(heading=270, speed=12)

# 위험도 평가
risk = risk_assessor.assess(
    os_position=(0, 0),
    os_velocity=os_velocity,
    ts_position=(1000, 500),
    ts_velocity=ts_velocity
)

print(f"Risk Level: {risk.risk_level.name}")
print(f"DCPA: {risk.dcpa:.0f} m")
print(f"TCPA: {risk.tcpa:.0f} s ({risk.tcpa/60:.1f} min)")
print(f"Current Distance: {risk.distance:.0f} m")
print(f"Bearing Rate: {risk.bearing_rate:.4f} deg/s")

# 위험 여부 확인
if risk.is_dangerous:
    print("⚠️ 위험 상황!")
    action = risk_assessor.get_recommended_action(risk)
    print(f"권장 조치: {action}")
```

**출력 예시**:
```
Risk Level: HIGH
DCPA: 500 m
TCPA: 90 s (1.5 min)
Current Distance: 1118 m
Bearing Rate: 0.1234 deg/s
⚠️ 위험 상황!
권장 조치: 즉시 회피 조치 필요. DCPA 500m, TCPA 90s...
```

---

### 3. 다중 목표선 평가

```python
from colregs_core import RiskAssessment, heading_to_velocity

risk_assessor = RiskAssessment()

# OS 정보
os_position = (0, 0)
os_velocity = heading_to_velocity(0, 10)

# 여러 TS 정보
targets = [
    ((3000, 3000), heading_to_velocity(180, 10)),  # TS1: 멀리
    ((500, 500), heading_to_velocity(180, 10)),    # TS2: 가까움 (위험)
    ((1000, 1000), heading_to_velocity(270, 12))   # TS3: 중간
]

# 가장 위험한 선박 찾기
result = risk_assessor.get_most_dangerous_target(
    os_position, os_velocity, targets
)

if result:
    target_idx, most_dangerous = result
    print(f"Most dangerous: Target {target_idx+1}")
    print(f"Risk: {most_dangerous.risk_level.name}")
    print(f"DCPA: {most_dangerous.dcpa:.0f}m")
else:
    print("All targets are safe")

# 모든 목표선 평가 (위험도 순)
risks = risk_assessor.assess_multiple_targets(
    os_position, os_velocity, targets
)

for i, risk in enumerate(risks):
    print(f"{i+1}. Risk: {risk.risk_level.name}, "
          f"DCPA: {risk.dcpa:.0f}m, TCPA: {risk.tcpa:.0f}s")
```

---

### 4. 통합 사용 (Encounter + Risk)

```python
from colregs_core import (
    EncounterClassifier,
    RiskAssessment,
    heading_to_velocity
)

# 초기화
classifier = EncounterClassifier()
risk_assessor = RiskAssessment()

# Ship 정보
os_pos = (0, 0)
os_hdg = 0
os_spd = 10
ts_pos = (1000, 1000)
ts_hdg = 270
ts_spd = 12

# 1. Encounter 분류
situation = classifier.classify(
    os_pos, os_hdg, os_spd,
    ts_pos, ts_hdg, ts_spd
)

# 2. 위험도 평가
os_vel = heading_to_velocity(os_hdg, os_spd)
ts_vel = heading_to_velocity(ts_hdg, ts_spd)
risk = risk_assessor.assess(os_pos, os_vel, ts_pos, ts_vel)

# 3. 종합 판단
print(f"=== Situation Analysis ===")
print(f"Encounter: {situation.encounter_type.value}")
print(f"Distance: {situation.distance:.0f}m")
print(f"Risk: {risk.risk_level.name}")

if risk.requires_action:
    print(f"\n⚠️ Action Required!")
    print(f"COLREGs: {classifier.get_action_requirement(situation.encounter_type)}")
    print(f"Tactical: {risk_assessor.get_recommended_action(risk)}")
```

---

## API Reference

### EncounterType (Enum)

조우 상황 타입:
- `HEAD_ON`: Rule 14 정면 조우
- `OVERTAKING`: Rule 13 추월
- `CROSSING_GIVE_WAY`: Rule 15 횡단 (OS가 피항선)
- `CROSSING_STAND_ON`: Rule 15 횡단 (OS가 유지선)
- `SAFE`: 안전 거리 밖
- `UNDEFINED`: 분류 불가

### RiskLevel (Enum)

위험도 등급:
- `SAFE` (0): 위험 없음
- `LOW` (1): 낮은 위험
- `MEDIUM` (2): 중간 위험
- `HIGH` (3): 높은 위험
- `CRITICAL` (4): 긴급 상황

### EncounterSituation (NamedTuple)

조우 상황 분석 결과:
```python
EncounterSituation(
    encounter_type: EncounterType,
    relative_bearing: float,    # degrees [0, 360)
    relative_course: float,     # degrees (-180, 180]
    distance: float,            # meters
    aspect_angle: float         # degrees [0, 360)
)
```

### CollisionRisk (NamedTuple)

충돌 위험 평가 결과:
```python
CollisionRisk(
    dcpa: float,              # meters
    tcpa: float,              # seconds
    risk_level: RiskLevel,
    distance: float,          # meters
    bearing_rate: float       # deg/s
)

# Properties:
risk.is_dangerous          # bool: Risk >= HIGH
risk.requires_action       # bool: Risk >= MEDIUM
```

---

## Geometry Utilities

### 기본 계산 함수

```python
from colregs_core.geometry import (
    normalize_angle,
    calculate_relative_bearing,
    calculate_distance,
    heading_to_velocity,
    velocity_to_heading_speed,
    calculate_aspect_angle,
    calculate_bearing_rate
)

# 각도 정규화
angle = normalize_angle(370)  # → 10

# 상대 방위각
rel_brg = calculate_relative_bearing(
    os_position=(0, 0),
    os_heading=0,
    ts_position=(1000, 1000)
)  # → 45.0

# 거리
dist = calculate_distance((0, 0), (1000, 1000))  # → 1414.2

# Heading → Velocity
velocity = heading_to_velocity(heading=45, speed=10)
# → (7.07, 7.07)

# Velocity → Heading
heading, speed = velocity_to_heading_speed((7.07, 7.07))
# → (45.0, 10.0)

# Aspect angle (TS에서 OS를 보는 방위)
aspect = calculate_aspect_angle(
    ts_heading=270,
    os_position=(0, 0),
    ts_position=(1000, 0)
)

# 방위각 변화율
br = calculate_bearing_rate(
    os_position=(0, 0),
    os_velocity=(10, 0),
    ts_position=(1000, 1000),
    ts_velocity=(0, -10)
)
```

### CPA/TCPA 계산

```python
from colregs_core.risk import (
    calculate_cpa_tcpa,
    calculate_cpa_position,
    is_collision_course,
    calculate_bow_crossing_range
)

# CPA/TCPA
dcpa, tcpa = calculate_cpa_tcpa(
    os_position=(0, 0),
    os_velocity=(10, 0),
    ts_position=(1000, 1000),
    ts_velocity=(0, -10)
)

# CPA 시점의 위치
os_cpa, ts_cpa, tcpa = calculate_cpa_position(
    os_position, os_velocity,
    ts_position, ts_velocity
)

# 충돌 코스 여부
is_danger = is_collision_course(
    dcpa=300,
    tcpa=600,
    dcpa_threshold=500,
    tcpa_threshold=1200
)

# Bow Crossing Range
bcr = calculate_bow_crossing_range(
    os_position, os_velocity,
    ts_position, ts_velocity
)
```

---

## 고급 사용 예제

### 예제 1: 실시간 감시 루프

```python
import time
from colregs_core import EncounterClassifier, RiskAssessment

classifier = EncounterClassifier()
risk_assessor = RiskAssessment()

def monitor_target(os_state, ts_state):
    """실시간 목표선 감시"""
    situation = classifier.classify(*os_state, *ts_state)
    
    os_vel = heading_to_velocity(os_state[1], os_state[2])
    ts_vel = heading_to_velocity(ts_state[1], ts_state[2])
    
    risk = risk_assessor.assess(
        os_state[0], os_vel,
        ts_state[0], ts_vel
    )
    
    if risk.is_dangerous:
        print(f"⚠️ ALERT: {situation.encounter_type.value}")
        print(f"   Risk: {risk.risk_level.name}")
        print(f"   DCPA: {risk.dcpa:.0f}m, TCPA: {risk.tcpa:.0f}s")
        return True
    
    return False

# 시뮬레이션 루프
while True:
    os_state = get_own_ship_state()
    ts_state = get_target_ship_state()
    
    if monitor_target(os_state, ts_state):
        trigger_alarm()
    
    time.sleep(1.0)  # 1초마다 업데이트
```

### 예제 2: DRL 환경 통합

```python
from colregs_core import EncounterClassifier, RiskAssessment

class NavigationEnv:
    def __init__(self):
        self.classifier = EncounterClassifier()
        self.risk_assessor = RiskAssessment()
    
    def get_observation(self):
        """Observation에 encounter type 포함"""
        obs = {}
        
        for ts in self.target_ships:
            situation = self.classifier.classify(
                self.os_position,
                self.os_heading,
                self.os_speed,
                ts.position,
                ts.heading,
                ts.speed
            )
            
            # Encounter type을 one-hot encoding
            obs[f'ts_{ts.id}_encounter'] = self._encode_encounter(
                situation.encounter_type
            )
            
            # 위험도 평가
            risk = self.risk_assessor.assess(...)
            obs[f'ts_{ts.id}_risk_level'] = risk.risk_level.value
        
        return obs
    
    def calculate_reward(self):
        """Encounter type별 차등 reward"""
        reward = 0
        
        for ts in self.target_ships:
            situation = self.classifier.classify(...)
            risk = self.risk_assessor.assess(...)
            
            # Encounter type별 가중치
            if situation.encounter_type == EncounterType.CROSSING_GIVE_WAY:
                reward -= risk.risk_level.value * 2.0  # 피항선 책임 강조
            elif situation.encounter_type == EncounterType.HEAD_ON:
                reward -= risk.risk_level.value * 1.5
            
            # COLREGs 준수 보너스
            if self._is_following_colregs(situation):
                reward += 1.0
        
        return reward
```

---

## 설정 커스터마이징

### 안전 거리 조정

```python
# 해역 특성에 따라 조정
classifier_coastal = EncounterClassifier(
    safe_distance=1000.0  # 연안 해역
)

classifier_open_sea = EncounterClassifier(
    safe_distance=3000.0  # 외해
)
```

### 위험도 임계값 조정

```python
# 선박 크기나 조종성능에 따라 조정
risk_assessor_large = RiskAssessment(
    dcpa_critical=300.0,   # 대형선
    dcpa_high=800.0,
    tcpa_critical=600.0
)

risk_assessor_small = RiskAssessment(
    dcpa_critical=150.0,   # 소형선
    dcpa_high=400.0,
    tcpa_critical=300.0
)
```

---

## 좌표계 주의사항

### 입력 좌표계:
- **Position**: (x, y) in meters
  - x: East (동쪽 양수)
  - y: North (북쪽 양수)
  
- **Heading**: degrees
  - 0° = North (정북)
  - 90° = East (정동)
  - 180° = South (정남)
  - 270° = West (정서)
  - Clockwise (시계 방향)

- **Velocity**: (vx, vy) in m/s
  - vx: East 방향 속도 성분
  - vy: North 방향 속도 성분

### 단위 변환:
```python
# Knots → m/s
speed_ms = speed_knots * 0.51444

# m/s → Knots
speed_knots = speed_ms * 1.944

# NM → meters
distance_m = distance_nm * 1852

# meters → NM
distance_nm = distance_m / 1852
```

---

## 트러블슈팅

### Q: Overtaking이 제대로 분류되지 않습니다.

**A**: Aspect angle과 속도 비교를 확인하세요.
```python
# OS가 TS보다 빠른지 확인
assert os_speed > ts_speed

# TS가 선미 섹터에 있는지
assert 112.5 <= relative_bearing <= 247.5
```

### Q: TCPA가 음수로 나옵니다.

**A**: 이미 CPA를 통과했다는 의미입니다. 현재 거리로 위험도를 평가합니다.

### Q: 위험도가 예상보다 높게/낮게 나옵니다.

**A**: 임계값을 조정하세요:
```python
risk_assessor = RiskAssessment(
    dcpa_critical=your_critical_distance,
    tcpa_critical=your_critical_time
)
```

---

## 다음 단계

1. `examples/integrated_example.py` 실행해보기
2. `tests/` 디렉토리의 테스트 코드 참고
3. `docs/colregs_rules.md` 에서 COLREGs 규칙 상세 확인
4. 프로젝트에 통합하여 사용

---

## 지원 및 기여

문의사항이나 버그 리포트는 이슈로 남겨주세요.
