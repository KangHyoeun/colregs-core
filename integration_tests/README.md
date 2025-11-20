# COLREGs-Core Integration Tests

강화학습 훈련 시작 전에 **colregs-core의 모든 기능이 정상 작동하는지 확인**하는 통합 테스트 스위트입니다.

## 🎯 테스트 목적

실제 Otter USV 시뮬레이션 환경에서 다음을 검증합니다:

1. **Geometry 모듈**: 좌표계 변환 (`heading_speed_to_velocity`)
2. **Utils 모듈**: 거리 계산, cross-track error 계산
3. **Risk 모듈**: 충돌 위험도 계산 (Jeon & Chun 방법)
4. **Reward 모듈**: 보상 함수 계산 (Jeon's 6-component reward)
5. **Encounter 모듈**: COLREGs 조우 분류 (Rules 13-15)

## 📁 파일 구조

```
integration_tests/
├── __init__.py
├── test_colregs_integration.py      # 메인 통합 테스트
├── run_tests.sh                      # 테스트 실행 스크립트
└── README.md                         # 이 파일
```

## 🚀 실행 방법

### 방법 1: 통합 테스트 스크립트 실행

```bash
cd /home/hyo/colregs-core/integration_tests
conda activate DRL-otter-nav
chmod +x run_tests.sh
./run_tests.sh
```

### 방법 2: 직접 실행

```bash
cd /home/hyo/colregs-core
conda activate DRL-otter-nav
poetry run python integration_tests/test_colregs_integration.py
```

## 📊 테스트 내용

### Test 1: Single Step Test

한 시뮬레이션 스텝에서 모든 모듈의 계산 값을 모니터링합니다.

**출력 예시:**
```
📐 Testing Geometry Module
  Heading: 45.00° (deg)
  Speed: 1.5000 m/s
  Velocity vector: [1.0607, 1.0607] m/s
  ✅ Magnitude error: 0.000000 m/s

⚠️  Testing Risk Module (Collision Risk)
1️⃣  Jeon Collision Risk:
  CR value: 0.2345
  DCPA: 12.34 m
  TCPA: 8.56 s
  d_eff: 15.67 m

🎁 Testing Reward Module
  r_dist: 0.1234
  r_track: -0.0123
  r_speed: 0.0456
  Total efficiency: 0.1567
  Total safety: 0.0234
  🎯 Total Reward: 0.1801
```

### Test 2: Multi-Step Test

10 스텝 동안 연속적으로 계산하면서 추이를 모니터링합니다.

**출력 예시:**
```
Step 1/10
  Position: [0.00, -90.00]
  Distance to goal: 180.00 m
  Jeon CR: 0.1234
  Reward: 0.0567

Step 10/10
  Position: [2.34, -75.12]
  Distance to goal: 165.12 m
  Jeon CR: 0.2456
  Reward: 0.0789

Total reward: 0.6543
Max collision risk: 0.2456
```

## ✅ 검증 포인트

### 1. Geometry 모듈
- ✅ `heading_speed_to_velocity`의 벡터 크기 = 입력 속도
- ✅ 방향이 헤딩과 일치

### 2. Utils 모듈
- ✅ `distance` 계산이 유클리드 거리와 일치
- ✅ `cross_track_error`가 예상 범위 내

### 3. Risk 모듈
- ✅ CR 값이 0~1 범위 내
- ✅ DCPA/TCPA가 합리적
- ✅ Jeon CR과 Chun CR이 유사한 패턴

### 4. Reward 모듈
- ✅ 효율성 보상 (r_dist, r_track, r_speed)이 합리적
- ✅ 안전성 보상 (r_collision, r_course_change, r_speed_change)이 합리적
- ✅ 총 보상이 예상 범위 내

### 5. Encounter 분류
- ✅ Position region (R1~R6) 분류가 정확
- ✅ Heading region (TSR1~TSR6) 분류가 정확
- ✅ Encounter type (Head-on/Crossing/Overtaking) 판정이 COLREGs 준수

## 🐛 문제 발생 시

### 1. ModuleNotFoundError

```bash
# colregs-core 설치 확인
cd /home/hyo/colregs-core
poetry install

# 패키지 경로 확인
poetry run python -c "import colregs_core; print(colregs_core.__file__)"
```

### 2. irsim 에러

```bash
# irsim 경로 확인
echo $PYTHONPATH
# /home/hyo/ir-sim/ir-sim 이 포함되어 있어야 함
```

### 3. PythonVehicleSimulator 에러

```bash
# PVS 경로 확인
ls /home/hyo/PythonVehicleSimulator/src/
```

## 📝 다음 단계

통합 테스트 통과 후:

1. ✅ **단일 Imazu 시나리오 학습**
   ```bash
   cd /home/hyo/DRL-otter-navigation
   poetry run python robot_nav/otter_rl_train_CNNPPO_imazu_01_scratch.py
   ```

2. ✅ **TensorBoard로 학습 모니터링**
   ```bash
   tensorboard --logdir=runs
   ```

3. ✅ **학습된 모델 평가**
   ```bash
   poetry run python robot_nav/otter_rl_test_CNNPPO_imazu_01_scratch.py
   ```

## 📚 참고 자료

- **전도현 박사 논문**: 보상 함수, Ship Domain, CR 계산
- **Chun et al. (2021, 2024)**: 보상 구조, 하이퍼파라미터
- **Woo & Kim (2020)**: 격자 지도, CNN 구조
- **Sawada et al. (2020)**: Imazu 22 시나리오

---

**작성자**: Navigation Officer & DRL Developer  
**날짜**: 2024-11  
**프로젝트**: Otter USV Autonomous Collision Avoidance
