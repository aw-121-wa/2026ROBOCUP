# RoboChassis V2 Implementation Plan

**Goal:** Build a CMake-only STM32F750V8T6 mecanum chassis stack for four ZDT X42S/Emm motors and a JY60 IMU, with host-tested algorithms, DMA-based nonblocking I/O, 5 ms control, VOFA telemetry, and an explicit pyOCD flashing workflow.

**Architecture:** CubeMX-owned code remains in `Core`, `Drivers`, and `Middlewares`; new code is layered into `Config`, `BSP`, `Driver`, `Algorithm`, and `App`. Hardware-dependent adapters call HAL, while parsers and control algorithms remain host-testable C11 modules.

**Tech Stack:** C11, STM32 HAL, CMSIS-RTOS2/FreeRTOS, Arm GNU Toolchain, CMake/Ninja, CTest, pyOCD, PowerShell.

**Global Constraints:** Coordinate frame is +X forward, +Y left, +W counter-clockwise. Application code accesses UARTs through `pin_config.h`. Real-time code uses DWT time and contains no `HAL_Delay` or blocking UART transmit. USART2/JY60 uses circular DMA at 9600 baud; USART3/X42S uses TX DMA at 921600 baud; UART4 carries bounded-rate VOFA CSV. Generated CubeMX regions are not edited. Motor output is calibration-gated and defaults safe. X42S uses fixed `0x6B` protocol, four synchronous F6 frames followed by broadcast `00 FF 66 6B`; the undocumented `00 AA` aggregate mode is not enabled. Firmware builds ELF/HEX/BIN/map through CMake and flashing is never automatic.

## Task 1: Pure control and motion algorithms

- Add host CTest infrastructure.
- Add `Algorithm/mecanum`, `wheel_limit`, `pid`, `planner`, `imu_heading`, `odom`, and `lateral_comp` headers/sources.
- Add literal-vector tests for mecanum forward/inverse transforms, rotation-priority saturation, PID limits, short/long cosine profiles, yaw wrap/fusion, odometry integration, and lateral scaling.
- Follow RED/GREEN for every behavior and commit the passing task.

## Task 2: DWT, JY60, X42S, and hardware configuration

- Add `Config/pin_config.h`, `Config/motor_config.h`, and `Config/chassis_config.h` with safe calibration gating and motor IDs FL=2, FR=1, RL=3, RR=4.
- Add `BSP/bsp_dwt` with wrap-safe cycle conversion.
- Rewrite `Driver/jy60` as a HAL-independent streaming parser plus a circular-DMA adapter and freshness state machine.
- Add `Driver/zdt_x42s` protocol builders, latest-command-wins DMA transport, TX-complete callback, stop/enable/sync commands, and protocol-vector tests.
- Do not enable movement unless geometry, wheel direction, motor IDs, and lifted-wheel checks are acknowledged in configuration.
- Follow RED/GREEN and commit the passing task.

## Task 3: Chassis integration, telemetry, CMake, pyOCD, and documentation

- Add `App/chassis_control` with a 5 ms DWT-scheduled update pipeline, IMU/odometry/planner/heading/mecanum/limit/quantize/send order, fault-safe zeroing, and 2 ms-capable timing.
- Add nonblocking UART4 VOFA CSV telemetry at 50 Hz.
- Integrate only through CubeMX `USER CODE` regions in `Core/Src/main.c`, `Core/Src/freertos.c`, and UART callbacks.
- Update CMake to build host tests and firmware, produce ELF/HEX/BIN/map, and retain only GCC presets.
- Add `tools/flash_pyocd.ps1` requiring explicit target and probe UID, plus `docs/TESTING.md` and `docs/BRINGUP.md` with lifted-wheel staged validation.
- Run all host tests, Debug firmware build, repository scans for blocking real-time calls and Keil files, then commit.

## Task 4: Whole-branch review and verification

- Review all changes against this plan and the user-supplied phases.
- Fix critical/important findings once, rerun CTest and firmware build, verify artifacts and clean Git status.
- Report exact desktop commands for host tests, firmware build, probe discovery, flashing, and staged hardware tests.
