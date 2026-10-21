# firmware — 공통 OBS/EFF 장치 런타임

OBS와 EFF는 **각각 별도 보드**에서 실행합니다. 같은 하부 터렛을 쓰므로 UART 제어·각도 피드백·정지 확인과
공용 ICD 1.0 통신은 `turret_runtime` 라이브러리로 재사용합니다.
EFF 프로그램 `mfs_eff`는 공통 라이브러리를, OBS 프로그램 `mfs_obs`는 공통 라이브러리와
`external/LiDAR` 원본 취득 라이브러리를 연결합니다. EFF에는 LiDAR SDK를 연결하지 않습니다.
각 보드에서는 프로그램 하나만 실행해 UART를 단독으로 소유합니다. 공통부는 독립 라이브러리이며
별도 상주 프로세스가 아니므로 보드 내부 IPC나 새로운 통신 규격이 필요하지 않습니다.
기존 `motor/motor-demo` 문자열 UDP 프로그램과 TCP 뷰어용 `live_sender.cpp`는 별도 legacy 실행 경로입니다.
새 런타임은 레이저 점등/출력 명령을 제공하지 않습니다.

## 포함된 기능

- C2 발급 접속 세션 등록, 기능·설치·현재각·Heartbeat 보고.
- EFF: CONTROL_OPEN, POINTING_COMMAND POINT/STOP/GET_STATUS/DRAIN.
- OBS: OBS_MODE_COMMAND FIXED_VIEW, OBS_MOTION_COMMAND의 유한 POINT/STOP/GET_STATUS/DRAIN,
  MEASUREMENT_COMMAND START/STOP, OBS_POSE, POINT_CLOUD_CHUNK.
- 같은 요청 재전송은 재실행하지 않고 캐시한 결과 반환. 본문 충돌은 ID_CONFLICT.
  접속 세션 내 epoch/commandSequence 최고값은 캐시 만료 후에도 유지합니다.
- 실제 피드백이 목표 범위에서 안정화된 뒤 COMPLETED. 접수만 된 상태는 ACCEPTED.
- STOP은 새로운 UART 읽기로 현재 위치를 확인하고 hold합니다. 이전 이동 재실행을 차단합니다.
- SDK 시작/정지는 별도 작업에서 처리해 통신을 막지 않고 실제 SDK 장치 상태까지 확인합니다.
- LiDAR는 무효 픽셀을 포함한 모든 원본 슬롯과 SDK 시각을 전달합니다. 장치에서 객체 검출하지 않습니다.

## 구성

```text
firmware/
  apps/eff.cpp           EFF 실행 진입점 (mfs_eff)
  apps/obs.cpp           OBS 실행 진입점, LiDAR backend 주입 (mfs_obs)
  icd/runtime.hpp         공통 런타임 API와 취득부 주입 경계
  icd/runtime.cpp         공용 UDP/Protobuf 수신·OBS/EFF 제어
  icd/motor.hpp           공통 터렛 제어·각도 보정·실측 피드백
  motor/                 기존 UART 드라이버 재사용 (timeout·checksum·읽기 유효성 보완)
  external/warwarden-protocol/  공용 proto, 935fc4f 고정
  external/LiDAR/capture/       SDK 원본 취득과 시뮬레이션 backend
```

공용 proto에서 C++ 코드를 생성하며 수동 wire codec이나 새로운 네트워크 형식은 추가하지 않습니다.
Python은 시험용 C2에만 필요하며 보드의 장치 런타임은 C++입니다.

## 빌드

CMake 3.18+, C++17, Protobuf C++/protoc 3.15+가 필요합니다.
Linux 개발 환경에서는 `cmake`, `g++`, `libprotobuf-dev`, `protobuf-compiler`를 준비합니다.

```sh
git submodule update --init --recursive
cmake -S . -B build/native -DLIDAR_WITH_XTSDK=OFF
cmake --build build/native --parallel
ctest --test-dir build/native --output-on-failure
```

Windows 시뮬레이션 빌드에는 같은 버전의 Protobuf C++ 라이브러리와 protoc를 준비하고
`-DCMAKE_PREFIX_PATH=...`를 지정합니다. 실행 파일은 `build/native/Release/mfs_obs.exe`, `mfs_eff.exe`입니다.
실제 UART backend는 Linux에서만 빌드합니다.
EFF만 빌드할 때는 `-DBUILD_OBS=OFF`를 지정합니다. 이 경우 LiDAR 구현이나 SDK 없이 빌드됩니다.
`runtime.hpp`의 `run_device()`를 통해 공통 런타임을 사용하고, OBS 진입점이 `CaptureFactory`로
취득 구현을 공급하므로 다른 센서 backend로 교체할 때 UART 제어부를 복제할 필요가 없습니다.

실제 OBS는 SDK 라이브러리까지 연결합니다.

```sh
cmake -S . -B build/board \
  -DLIDAR_WITH_XTSDK=ON \
  -DXTSDK_INCLUDE_DIR=/path/to/xtsdk \
  -DXTSDK_LIBRARY=/path/to/libxtsdk.a \
  -DXTSDK_EXTRA_LIBRARIES="boost_system;boost_filesystem;opencv_core;opencv_imgproc;dl;pthread"
cmake --build build/board --parallel
```

PetaLinux ARM 대상은 해당 이미지의 CMake toolchain/sysroot와 **ARM용** Protobuf·XT SDK 라이브러리를 사용합니다.
코드 생성에 실행하는 protoc는 빌드 호스트용이어야 합니다. Windows x64 바이너리를 보드에 복사해 실행하지 않습니다.

## 현재 C2와 시뮬레이션

C2 저장소의 기존 콘솔을 먼저 띄우고 다음 두 장치를 각각 실행합니다.

```text
mfs_obs --simulate --c2-address 127.0.0.1 --c2-port 5000
mfs_eff --simulate --c2-address 127.0.0.1 --c2-port 5000
```

기본 식별자는 OBS=101, EFF=201입니다. C2의 `configs/team-demo.json`과 맞습니다.
콘솔에서 `targets`, `aim 201`, `status`, `stop 201`, `collect stop`, `collect start`, `quit`을 시험합니다.
`--simulate`는 합성 프레임·모터 모델을 사용하는 localhost 전용 모드입니다.

OBS 동작 순서는 **수집 중지 완료 → 유한 이동 → 실측 각도 안정화·정지 확인 → 수집 시작**입니다.
OBS_MOTION_COMMAND의 ACCEPTED만으로 수집하지 않습니다. UART 피드백이 목표 오차 범위에서
`--settle-ms` 동안 안정화되어 COMPLETED가 된 뒤 MEASUREMENT START를 보냅니다.
정지가 확인되기 전의 START는 BUSY로 거부합니다. 이동 완료 자체가 자동 수집을 시작하지는 않습니다.
수집 중 OBS POINT는 BUSY로 거부합니다. FOLLOW_OBJECT/이동 중 연속 취득·보정은 이번 범위에 포함하지 않습니다.
C2 CLI의 `obs point PAN TILT`, `obs stop`, `obs status`로 조작합니다.
`obs point`는 EFF 정지와 수집 중지부터 순서대로 처리하고 정지 확인 후 수집을 재개합니다.
자동 지향은 측정된 OBS pan/tilt=0의 고정 관측만 허용합니다.

## 실물 설정

실물 모드에는 기본 축 보정값을 넣지 않습니다. 다음 값을 실제 확인해 명시해야 합니다.

| 옵션 | 의미 |
| --- | --- |
| `--uart`, `--baud` | 모터 제어 보드 UART와 속도 (기본 baud=115200) |
| `--calibration-id` | C2 설치 설정과 같은 축 보정 ID |
| `--pan-zero`, `--tilt-zero`, `--ticks-per-turn` | 축 0점과 tick/회전 |
| `--pan-sign`, `--tilt-sign` | 각도 증가 방향, +1 또는 -1 |
| `--pan-min-ticks`, `--pan-max-ticks`, `--tilt-min-ticks`, `--tilt-max-ticks` | 실제 확인한 기구 한계 |
| `--point-timeout-ms`, `--stop-timeout-ms` | 실제 완료 확인 제한시간 |
| `--settle-ms`, `--tolerance-deg` | 안정화 시간과 허용 현재각 오차 |
| `--clock-verified`, `--clock-uncertainty-ms` | 외부에서 확인한 보드 OS 시간 동기화와 오차 상한 |
| OBS: `--sensor-ip`, `--board-ip`, `--sensor-profile` | 실제 센서·수신 보드 IP와 SDK 설정 프로파일 |

시각 검증을 제공하지 않으면 clockState=UNKNOWN입니다. 실행 중이라는 이유로 SYNCED로 기본 보고하지 않습니다.
UART 읽기 실패는 축별 invalid로 표시하며 이전 각도·명령각·0으로 대체하지 않습니다.

## 취득 시각·자세 연결

실물 OBS는 검증된 `--acquisition-profile configs/acquisition-profile.json`을 제공하면 SDK 원시 시각을
UTC로 매핑하고, 불확실성을 포함한 전체 노출 구간을 UART 실측 이력의 앞뒤 샘플로 확인합니다.
각 축의 시간·유효성·연속성·정지 상태와 이동 세대가 일치해야 유효한 OBS_POSE를 보냅니다.
확인할 뒤쪽 샘플이 아직 없으면 잠시 기다리며, 이력이 부족하거나 이동/오류/시간 변경이 있으면 승인하지 않습니다.

프로파일은 센서 IP·설정 ID·현재 Linux boot ID·증빙·유효기간에 묶입니다. 예제의 null/기한 만료 값은
의도적으로 실행 불가능하며, 실측으로 채워야 합니다. `uncertainty_ms`에는 센서 매핑과 보드 시계,
밀리초 직렬화 반올림 오차까지 포함합니다. 콜백 수신 시각이나 명령 각도로 대체하지 않습니다.
C2에서는 같은 파일을 `clock-profile 101 profile.json`으로 현재 장치 boot/session에 연결합니다.

프로파일이 없거나 조건이 맞지 않으면 원시 점군을 계속 보내되 `pose_valid=false`이며 C2는 DISPLAY_ONLY로
처리합니다. 하드웨어에서 프로파일의 값 자체가 맞는지는 아직 검증하지 않았습니다.
현재 C2는 유효한 프레임이라도 OBS 각도가 비영점이면 자동 aim에 사용하지 않습니다.
[보드 검증 문서](docs/BOARD_VALIDATION.md)에 각 필드와 설정 순서를 정리했습니다.

## 검증

C2 저장소의 Python 환경과 검출기 실행 파일을 사용합니다.

```text
C2_PYTHON tests/test_c2_link.py --obs build/native/mfs_obs --eff build/native/mfs_eff --c2-root ../ai-generated-c2 --detector ../ai-generated-c2/build/c2_detector_worker
```

Windows 경로 예시:

```powershell
../ai-generated-c2/.venv-team/Scripts/python.exe tests/test_c2_link.py --obs build/icd/Release/mfs_obs.exe --eff build/icd/Release/mfs_eff.exe --c2-root ../ai-generated-c2
```

- 실제 UDP: Python C2 ↔ 서로 다른 C++ OBS/EFF 실행 파일과 별도 프로세스.
- OBS 이동 접수 직후 수집 요청 거부, 실측 정지 완료 후 수집 허용, 수집 중 이동 거부.
- 원본 수신·검출·연속 지향·정지·수집 재시작, OBS 프로세스 소실 시 EFF 정지.
- 중복 요청·ID_CONFLICT·STOP 이후 이전 순번 거부, DRAIN fence.
- SDK 대역 fixture: 전체 슬롯·무효점·SDK 시각 보존, 최신 프레임 하나, 실제 상태 확인.
- Linux 전용 PTY 시험: 실제 UART parser에 분할·역순·불량 checksum·미응답을 주입합니다.

Windows에서 테스트한 합성 backend와 실제 보드 검증은 구분합니다. 실제 UART·SDK·기구 보정 시험은 아직 하지 않았습니다.

## 보드 배포 확인

[BOARD_VALIDATION.md](docs/BOARD_VALIDATION.md)에 검증 범위, 남은 실물 조건, 보드에서 볼 명령·예상 결과를 정리했습니다.
firmware PR은 LiDAR capture PR의 커밋을 submodule로 참조하므로 LiDAR PR을 먼저 병합해야 합니다.
