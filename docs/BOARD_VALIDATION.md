# 보드 배포 전 검증 및 시연 절차

## 현재 판정

별도 OBS/EFF 실행 파일, 공통 터렛 제어, ICD 패킷, 원본 점군→C2 검출→aim→stop은
합성 장치로 검증합니다. **실물 보드에 복사하면 전체 자동 지향이 바로 동작하는 단계는 아닙니다.**
실물 보드·모터·LiDAR에는 이 작업에서 접속하지 않았습니다.

| 경로 | 확인 방법 | 의미와 한계 |
| --- | --- | --- |
| 원본 점군→검출→aim 반복→stop | Python C2와 C++ OBS/EFF를 별도 프로세스로 실행 | 실제 UDP·공용 protobuf·검출기를 통과. 센서와 모터는 합성 |
| OBS 이동 후 수집 | 이동 ACCEPTED 직후 START 거부, COMPLETED 후 START 허용 | 정지 확인을 생략하지 않음. C2 콘솔에는 OBS 이동 명령이 아직 없음 |
| UART 패킷·실측 응답 | Linux PTY와 실제 motor.cpp | 쓰기 바이트·체크섬·분할/역순·불량/미응답 검증. 전기적 연결과 실모터 검증은 아님 |
| 실제 UART runtime | Linux PTY 모터 제어 보드 + mfs_eff + ICD 상대 | 이동·실측 완료·STOP, 피드백 소실 시 FAILED 확인 |
| SDK adapter | 실제 adapter 소스 + SDK fixture, 공식 헤더 컴파일 | 제조사 바이너리와 실센서 실행은 미검증 |
| ARM | EFF 전체 Ubuntu ARMHF 링크 및 UART/SDK adapter object 컴파일 | PetaLinux sysroot 링크·ABI·보드 부팅 검증은 아님 |

시험의 최종 통과 여부와 로그는 해당 PR의 CI checks에서 확인합니다.
C2 저장소는 비공개이므로 전체 C2↔장치 연동은 C2 저장소의 `Native device integration` workflow에서
실행합니다. 공개 firmware workflow에는 비공개 소스를 복사하거나 사용자 토큰을 저장하지 않습니다.
firmware workflow 자체에서는 UART/SDK 계약·독립 빌드·EFF ARMHF 링크를 실행합니다.
Windows에서는 Linux PTY 시험을 skip하며 합성 UDP 경로를 실행합니다.

## 실제 보드에서 아직 필요한 조건

1. 저장소 `vivado/servo_test.xpr`의 칩은 `xc7z010clg400-1`이고 기존 Makefile은
   `arm-linux-gnueabihf-g++`를 씁니다. 실제 이미지의 `uname -m`, `getconf LONG_BIT`와 대조합니다.
2. 해당 PetaLinux sysroot에 맞는 C++17 toolchain, ARM Protobuf 라이브러리, 호스트용 protoc가 필요합니다.
   CI는 Ubuntu ARMHF용 EFF 전체 링크를 확인합니다. PetaLinux sysroot용 전체 링크나 보드 실행을 대신하지 않습니다.
3. 확인한 [공식 SDK 커밋](https://github.com/XT-Toffuture/xtsdk_cpp/tree/3d3db067ae9bdc0528202c3087bc10fd3b706638/xtsdk/lib/linux)은
   x86_64/aarch64 제조사 바이너리만 제공합니다. **32비트 Zynq용 제조사 바이너리를 별도 확보해야 합니다.**
   ARM64 `.so`를 ARM32 보드에서 사용할 수 없습니다. 별도 공급본이 있다면 그 버전·ABI를 대조합니다.
4. `XTSDK_LIBRARY`는 `XinTan::XtSdk` wrapper (`libxtsdk.a` 등)와 그 의존성입니다.
   제조사 `libxtsdk_shared.so` 하나만 지정하는 것으로는 충분하지 않습니다.
   wrapper가 읽는 제조사 바이너리 경로와 Boost/OpenCV 등 런타임 의존성도 맞아야 합니다.
5. UART 장치·권한·baud·모터 ID(팬=1/틸트=2), sync-write 주소42/read 주소56,
   실제 위치 tick의 방향·0점·범위·속도200의 장비 의미를 확인합니다.
6. 현재 실물 adapter에는 **SDK 시각→UTC 매핑·노출 구간·그 구간의 UART 자세 연결이 미구현**입니다.
   실물 데이터는 원시 시각과 슬롯을 보내지만 `pose_valid=false`이며 C2에서 `DISPLAY_ONLY`로 남습니다.
   `--clock-verified`를 지정하거나 C2에 `clock-load`를 하는 것만으로 이 구현을 대신할 수 없습니다.
7. 현재 C2 자동 지향은 OBS 실측 pan/tilt=0 고정 관측만 허용합니다. 비영점 OBS 이동 후의
   자동 지향과 이동 중 취득은 이번 MVP에 포함하지 않습니다.
8. 실제 해상도·FPS의 점군 대역폭, UDP 누락률, 메모리·CPU와 조립시간 1초 충족은 보드에서 측정해야 합니다.
   8×6 합성 프레임 통과는 실제 해상도 처리량을 증명하지 않습니다.

## 준비 확인 명령 (모터 명령을 보내지 않음)

```sh
uname -m
getconf LONG_BIT
file ./mfs_eff ./mfs_obs
ldd ./mfs_eff
ldd ./mfs_obs
ls -l /dev/ttyPS* /dev/ttyUSB*
./mfs_eff --help
./mfs_obs --help
```

보드 ABI와 실행 파일이 같고, `ldd`에 `not found`가 없으며, UART 경로·접근 권한이 있어야 합니다.
프로그램 실행 시에는 기존 motor-demo 등 같은 UART를 사용하는 프로그램을 먼저 종료합니다.
기본 실행만으로는 부족하며 README의 모든 실물 보정 옵션을 제공해야 합니다.

C2는 `configs/team-demo.json`을 복사해 `simulation=false`로 바꾸고,
각 보드와 같은 `calibration_id`, OBS `profile`, 실제 상대 위치·장착 회전을 기록합니다.
합성 보정 ID나 임의 위치를 실측값처럼 쓰지 않습니다. 설치 revision은 설정 변경 시 갱신합니다.
C2 수신 바인드는 기본 localhost에서 실제 보드에 도달 가능한 주소로 바꿉니다.

```sh
python scripts/run_team_mvp.py --config configs/board.json --bind 0.0.0.0 --asset-port 5000 --detector /path/to/c2_detector_worker
```

보드 실행에는 `--c2-address <C2의 LAN IP> --c2-port 5000`을 사용합니다.
센서 SDK UDP 포트7687과 C2 UDP5000의 경로를 확인합니다. 실제 SDK 설정 프로파일에서
전체 픽셀 슬롯·XYZ 단위/축·필터·ROI·다운샘플 설정도 확인해야 합니다.

## 사용자가 단계별로 확인할 수 있는 결과

| 단계 | 실행/관찰 | 기대 결과 |
| --- | --- | --- |
| 더미 시연 | localhost에서 mfs_obs/mfs_eff `--simulate`, 콘솔 `targets` | observation CURRENT, 프레임과 검출 물체 목록 갱신 |
| 더미 지향 | `aim 201`, 이어서 `status` | EFF의 POINT 로그가 반복되고 각도/명령 순번이 바뀜 |
| 더미 정지 | `stop 201`, 이어서 `status` | STOPPING 후 STOPPED, 이후 POINT 증가 중단 |
| 수집 제어 | `collect stop`, `collect start` | 정지 결과 후 새 collection session으로 수집 재개, aim은 자동 재개하지 않음 |
| EFF 보드 등록 | 실물 설정으로 mfs_eff 실행, `status` | 201 등록·설치 완료, 각 축 measured/valid 현재각 보고 |
| EFF 수동 이동 | 확인한 기구 범위의 작은 각도로 `point 201 PAN TILT` | 실제 축 이동 후 목표 범위에서 안정화, 그 뒤 완료. 각도와 실제 방향 대조 |
| EFF 정지 | `stop 201` | 새 UART 읽기 후 현재 위치 hold와 정지 확인. 물리 정지와 콘솔 상태 모두 대조 |
| UART 장애 | 모터 응답 실패를 시험 환경에서 주입 | invalid/FAULT 또는 명령 FAILED. 성공 각도·COMPLETED로 위장하지 않음 |
| OBS 보드 원본 수신 | SDK 준비 후 collect start, `targets`/`events` | 수집 상태와 프레임 수신 확인. 현 구현에서는 DISPLAY_ONLY가 예상됨 |
| 실물 자동 aim | 시각·자세 연결 구현과 검증 후 수행 | 현재 단계의 완료 항목이 아님 |

네트워크 Heartbeat가 2초 끊기면 새 명령 gate를 닫고 측정한 현재 위치 hold를 시도합니다.
다만 UART 단절·프로세스 강제종료·전원 소실에서 소프트웨어가 물리 정지를 보장하지는 않습니다.
STOP은 모터 전원 차단이 아니라 측정 위치 hold이며 레이저 점등/출력 제어는 추가하지 않았습니다.
