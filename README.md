# 👾 STM32 LCD 팩맨 미니게임

---
## 💡 1. 프로젝트 개요

STM32 보드에서 **조이스틱 입력**으로 펙맨을 움직이도록 하고 **16x2 LCD에서 게임을 진행**하는 미니게임 프로젝트입니다.

**ADC + DMA 기반 조이스틱 방향 판별**, **TIM 인터럽트 기반 적 이동 타이밍 제어**, **I2C LCD 커스텀 캐릭터(CGRAM) 렌더링**을 적용하였고,
**(1) 레벨업(속도 증가), (2) 먹이 시스템(전체 칸 방문), (3) PWM 효과음**을 중심으로 한 **STM32 게임 애플리케이션**을 구현했습니다.

<img src="./images/pecman_gif.gif" width="400" height="300" alt="image">

---
## 🧭 2. 전체 구조 및 아키텍처


### 📍 구성 요소 설명

* **Joystick Input (ADC1 + DMA)**

  * 역할: 사용자의 방향 입력 수집
  * 핵심 기능: ADC 2채널을 DMA로 연속 수집하여 X/Y 값을 `dir[0], dir[1]`에 저장
  * 입력 / 출력: (조이스틱 전압) → (ADC 변환값) → `Dir_Joystick()` 방향(UP/DOWN/LEFT/RIGHT/NONE)

* **Game Logic (State Machine + Collision/Level)**

  * 역할: 게임 상태(진행/승리/패배), 캐릭터 이동, 레벨업 판정
  * 특징: `ING/WIN/OVER` 상태 머신 기반, 16x2 전체 칸(32칸) 방문 완료 시 레벨 업/승리 처리

* **Display & Sound (I2C LCD + TIM3 PWM)**

  * 역할: LCD에 캐릭터/먹이/문구 렌더링, 효과음 출력
  * 사용자와의 상호작용: LCD 화면에서 팩맨/문어 위치 확인, 먹이(미방문 칸) 수집, 승/패 메시지 확인

---

### 🔗 데이터 / 통신 흐름

* **ADC(DMA)** → **Dir_Joystick()** : 조이스틱 X/Y 변환값 기반 방향 결정
* **Direction** → **Move_Pacman()** : 팩맨 위치(row/col) 업데이트 + 방문 배열 기록
* **TIM2 Interrupt(clk_pulse)** → **Move_Enemy()** : 토글 타이밍에 맞춰 문어 이동
* **GameStatus()** → **LCD / Sound** : 충돌/클리어/레벨업 여부에 따라 메시지 및 효과음 출력

---

## 🛠 3. 기술 스택

### 🔹 Core Technologies

* Language: **C**
* Framework / Library: **STM32 HAL**, (LCD) **I2C LCD Driver (`I2C_lcd.h`)**
* Platform / Environment: **STM32 MCU**, 16x2 Character LCD(I2C), Joystick(Analog)

### 🔹 Tools & Infrastructure

* Database: 없음
* Communication / API: **I2C**, **UART(초기화 포함)**, **ADC(DMA)**, **TIM Interrupt**, **PWM**
* DevOps / Tools: **STM32CubeIDE / CubeMX(일반적 구성)**, **Git**

---

## ⭐ 4. 주요 기능 상세

### 1) 조이스틱 기반 팩맨 이동 (ADC + DMA)

* ADC1을 2채널로 구성하여 조이스틱 X/Y를 읽고 DMA로 지속 수집
* `Dir_Joystick()`에서 임계값(예: 600/3150)을 기준으로 방향을 판정
* `Move_Pacman()`에서 LCD 범위(16x2: row 0~1, col 0~15)를 넘어가지 않도록 경계 처리
* 입력 → 처리 → 출력:

  * 조이스틱 → ADC(DMA) `dir[]` → Direction → (row/col 갱신) → LCD 캐릭터 표시

---

### 2) 먹이(방문) 시스템 + 사운드 트리거

* `past_position[2][16]` 배열로 “방문 여부”를 관리

  * 방문한 칸 = 1, 미방문 칸 = 0
* `LCD_Display_Charactor()`에서

  * 미방문 칸에는 먹이(0xA5)를 출력
  * 방문한 칸은 공백(0x20)으로 overwrite
* 먹이 개수 `count`가 줄어드는 순간(= 먹이를 먹음)에 PWM 듀티를 주어 “먹는 소리” 출력

---

### 3) 문어(Enemy) 추적 이동 + 레벨업(속도 증가)

* TIM2 주기 인터럽트에서 `clk_pulse` 토글
* `Move_Enemy()`는 `clk_pulse` 변화 시점에만 이동(중복 이동 방지: `clock_before` 사용)
* 이동 방식:

  * 랜덤 선택(0/1)으로 “행 맞추기” 또는 “열 방향 추적”
* 레벨업 조건:

  * 32칸 모두 방문 시 레벨업(LEVEL 1 → 2 → 3)
  * 레벨업 시 `TIM2->PSC` 값을 낮춰 문어 이동 속도 증가
* 승/패:

  * 팩맨과 문어 좌표가 같으면 `OVER` + 패배 사운드
  * LEVEL3에서 32칸 완료 시 `WIN` + 승리 사운드

---

## 👨‍💻 5. 담당 역할

### 🔹 개인 기여도 요약

* 프로젝트 인원: **2명**
* 본인 역할: **임베디드 펌웨어(입력/출력/타이머/게임 로직)**

---

### 🔹 상세 담당 업무

#### ① 게임 로직/상태 머신 설계

* `ING/WIN/OVER` 상태 기반 메인 루프 구성
* 전체 칸 방문(32)을 승리/레벨업 조건으로 정의하고 `GameStatus()`로 통합 관리

#### ② 주변장치 연동(ADC DMA / I2C LCD / Timer / PWM)

* ADC1 2채널 + DMA로 조이스틱 값을 안정적으로 수집
* LCD CGRAM에 커스텀 캐릭터(팩맨 4프레임 + 적 1개) 등록 및 렌더링 구현
* TIM2 인터럽트로 적 이동 타이밍 제어, TIM3 PWM으로 시작/승리/패배/먹이 효과음 구현

#### ③ 시스템 관점 기여

* 입력(조이스틱) → 로직(이동/판정) → 출력(LCD/사운드) 흐름을 하나의 애플리케이션으로 통합
* 타이밍(인터럽트)과 화면 갱신(오버라이트) 간 충돌을 최소화하도록 루프 딜레이 및 출력 순서 조정

