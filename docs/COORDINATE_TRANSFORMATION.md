# 좌표계 변환 레이어 (Coordinate Transformation Layer)

## 📐 개요

ir-sim과 colregs-core 사이의 좌표계 차이를 해결하기 위한 변환 레이어입니다.

## 🎯 좌표계 정의

### 1. **ir-sim 좌표계** (일반 로봇공학 좌표계)
```
         +y (90°)
          ↑
          |
          |
-x ← - - -+- - - → +x (0°)
(180°)    |      
          |
          ↓
         -y (270°)
```

- **0° = East** (+x 방향)
- **90° = North** (+y 방향)
- **180° = West** (-x 방향)
- **270° = South** (-y 방향)
- **회전 방향**: 반시계방향 (CCW)
- **속도 변환**: `vx = v*cos(θ)`, `vy = v*sin(θ)`

### 2. **Navigation 좌표계** (항해 좌표계)
```
         North (0°)
              ↑ +y
              |
              |
West ← - - - -+- - - - → East
(270°)        |        (90°)
              |
              ↓ +x
         South (180°)
```

- **0° = North** (+y 방향)
- **90° = East** (+x 방향)
- **180° = South** (-y 방향)
- **270° = West** (-x 방향)
- **회전 방향**: 시계방향 (CW)
- **속도 변환**: `vx = v*sin(θ)`, `vy = v*cos(θ)`

## 🔄 변환 규칙

### Heading 변환

| ir-sim | Navigation | 방향 |
|--------|-----------|------|
| 0° | 90° | East |
| 90° | 0° | North |
| 180° | 270° | West |
| 270° | 180° | South |

**변환 공식**:
```python
nav_heading = (90 - irsim_heading) % 360
irsim_heading = (90 - nav_heading) % 360
```

### 속도 벡터

속도 벡터 자체는 변환 없이 그대로 사용됩니다. 두 좌표계 모두 동일한 (x, y) 평면을 사용하므로, 속도 벡터는 동일하게 유지됩니다. **단지 heading의 해석만 다릅니다.**

## 📦 API 문서

### `irsim_to_nav_heading(irsim_heading: float) -> float`

ir-sim heading을 Navigation heading으로 변환합니다.

**Args:**
- `irsim_heading`: ir-sim heading (degrees, 0=East, CCW)

**Returns:**
- `nav_heading`: Navigation heading (degrees, 0=North, CW)

**Example:**
```python
from colregs_core import irsim_to_nav_heading

# ir-sim에서 동쪽을 향함 (0°)
nav_heading = irsim_to_nav_heading(0)
print(nav_heading)  # 90° (Navigation에서 동쪽)

# ir-sim에서 북쪽을 향함 (90°)
nav_heading = irsim_to_nav_heading(90)
print(nav_heading)  # 0° (Navigation에서 북쪽)
```

### `nav_to_irsim_heading(nav_heading: float) -> float`

Navigation heading을 ir-sim heading으로 변환합니다.

**Args:**
- `nav_heading`: Navigation heading (degrees, 0=North, CW)

**Returns:**
- `irsim_heading`: ir-sim heading (degrees, 0=East, CCW)

**Example:**
```python
from colregs_core import nav_to_irsim_heading

# Navigation에서 북쪽을 향함 (0°)
irsim_heading = nav_to_irsim_heading(0)
print(irsim_heading)  # 90° (ir-sim에서 북쪽)
```

### `irsim_velocity_to_nav(irsim_heading: float, speed: float) -> Tuple[float, float]`

ir-sim heading과 속도를 속도 벡터로 변환합니다.

**Args:**
- `irsim_heading`: ir-sim heading (degrees)
- `speed`: 속도 크기 (m/s)

**Returns:**
- `(vx, vy)`: 속도 벡터 (m/s)

**Example:**
```python
from colregs_core import irsim_velocity_to_nav

# ir-sim에서 동쪽(0°)으로 10m/s
vx, vy = irsim_velocity_to_nav(0, 10)
print(f"vx={vx:.1f}, vy={vy:.1f}")  # vx=10.0, vy=0.0
```

### `verify_transformation() -> bool`

좌표계 변환이 올바른지 검증합니다.

**Returns:**
- `bool`: 모든 테스트 통과 시 True

**Example:**
```python
from colregs_core import verify_transformation

success = verify_transformation()
# ✓ 모든 좌표계 변환 테스트 통과
```

## 🚢 실제 사용 예시

### simple_maritime_simulation.py

```python
from colregs_core import (
    EncounterClassifier,
    irsim_to_nav_heading,
    irsim_velocity_to_nav
)

def get_sensor_data(self):
    """센서 데이터 수집 + 좌표계 변환"""
    
    # ir-sim에서 로봇 상태 가져오기
    robot_state = self.env.robot.state
    irsim_heading_deg = float(np.degrees(robot_state[2, 0]))
    
    # 🔄 좌표계 변환: ir-sim → Navigation
    nav_heading_deg = irsim_to_nav_heading(irsim_heading_deg)
    
    return {
        'robot_heading_irsim': irsim_heading_deg,  # ir-sim용
        'robot_heading_nav': nav_heading_deg,      # COLREGs용
        ...
    }

def analyze_encounters_from_sensor(self):
    """조우 상황 분석"""
    sensor_data = self.get_sensor_data()
    
    # COLREGs 분석 시 Navigation heading 사용
    situation = self.encounter_classifier.classify(
        os_position=os_position,
        os_heading=sensor_data['robot_heading_nav'],  # 🧭 Navigation
        os_speed=os_speed,
        ...
    )
```

## ✅ 테스트

좌표계 변환이 올바르게 작동하는지 테스트:

```bash
cd /home/hyo/colregs-core
python3 test_coordinate_transform.py
```

**예상 출력:**
```
============================================================
좌표계 변환 검증 (Coordinate Transformation Verification)
============================================================
✓ East        : ir-sim    0.0° → Nav   90.0° → ir-sim    0.0°
✓ North       : ir-sim   90.0° → Nav    0.0° → ir-sim   90.0°
✓ West        : ir-sim  180.0° → Nav  270.0° → ir-sim  180.0°
✓ South       : ir-sim  270.0° → Nav  180.0° → ir-sim  270.0°
✓ Northeast   : ir-sim   45.0° → Nav   45.0° → ir-sim   45.0°
✓ Northwest   : ir-sim  135.0° → Nav  315.0° → ir-sim  135.0°

속도 벡터 변환 검증:
✓ East        : ir-sim    0.0° → velocity ( 10.00,   0.00) m/s
✓ North       : ir-sim   90.0° → velocity (  0.00,  10.00) m/s
✓ West        : ir-sim  180.0° → velocity (-10.00,   0.00) m/s
✓ South       : ir-sim  270.0° → velocity (  0.00, -10.00) m/s
============================================================
✓ 모든 좌표계 변환 테스트 통과
============================================================
```

## 🎓 DRL 훈련 시 주의사항

1. **State Space**: ir-sim의 상태를 그대로 사용 가능
   - `robot.state[2, 0]`: ir-sim heading (radian)
   - DRL agent는 ir-sim 좌표계로 학습

2. **COLREGs 분석**: Navigation 좌표계 필요
   - `irsim_to_nav_heading()` 사용하여 변환
   - Encounter classification, risk assessment

3. **Action Space**: ir-sim 좌표계 사용
   - `u_ref`: surge velocity
   - `r_ref`: yaw rate (ir-sim 기준)

## 📚 참고 자료

- **ir-sim**: `/home/hyo/ir-sim/irsim/lib/algorithm/kinematics.py`
- **colregs-core**: `/home/hyo/colregs-core/src/colregs_core/geometry/`
- **변환 함수**: `coordinate_transform.py`
- **적용 예시**: `simple_maritime_simulation.py`

## ❓ FAQ

**Q: 왜 속도 벡터는 변환하지 않나요?**

A: 두 좌표계 모두 동일한 (x, y) 평면을 사용합니다. 속도 벡터 (vx, vy)는 물리적으로 동일하며, 오직 heading의 **해석**만 다릅니다.

**Q: DRL 훈련 시 어떤 좌표계를 사용해야 하나요?**

A: DRL agent는 **ir-sim 좌표계**로 학습합니다. COLREGs 분석이 필요할 때만 Navigation 좌표계로 변환하면 됩니다.

**Q: 기존 코드를 수정해야 하나요?**

A: `simple_maritime_simulation.py`는 이미 변환 레이어가 적용되어 있습니다. 새로운 코드 작성 시에만 변환 함수를 사용하면 됩니다.

---

**작성**: Maritime Robotics Lab  
**날짜**: 2025-10-27  
**버전**: 1.0
