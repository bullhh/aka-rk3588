# Orange Pi Robot Migration Plan

## 1. Background

Current local repository `aka-rk3588` is a C++ RK3588 robot program. Its main executable is `tennis`, and the current implementation already contains the core closed loop:

- UVC camera capture
- RKNN YOLOv8 tennis ball detection
- Differential chassis control
- Serial arm control
- Ball chasing, grabbing, bucket searching, bucket approach, and ball deposit state machine

The target Orange Pi device is:

- Host: `orangepi@10.3.10.24`
- Platform: RK3588 aarch64
- New development directory: `/home/orangepi/robot`

The Orange Pi already has an older project at:

```text
/home/orangepi/Code/Desktop-Wanderer
```

This project matches the current physical robot shape better than the local C++ repository. It uses a LeKiwi-style three omniwheel/mobile-base platform and Feetech STS3215 servo bus for the arm and base.

## 2. Current Repository Summary

Local repository:

```text
/home/szy/work/robot/tripod/aka-rk3588
```

Important files:

- `tennis.cpp`: Main C++ closed-loop state machine.
- `capture/uvc_capture.cpp`: UVC MJPEG camera capture.
- `detect/detect.cpp`: RKNN YOLOv8 inference wrapper.
- `motor/`: Old motor abstraction, using UART/PWM differential wheel control.
- `arm/`: Old ZP10D serial servo arm control.
- `test_cmds.cpp`: Test commands for UVC, YOLO, motor, arm, and bucket detection.
- `CMakeLists.txt`: RK3588 build configuration.

Main behavior:

1. Search for a tennis ball.
2. Chase and align with the ball.
3. Stop and grab the ball.
4. Search for a red bucket.
5. Approach the bucket.
6. Deposit the ball.
7. Return to ball search.

Important limitation for the new Orange Pi robot:

- `motor/` is built around left/right differential wheel control and does not match the current three-wheel/three-foot LeKiwi form.
- `arm/` is built around old ZP10D serial servo commands and does not match the current Feetech STS3215 bus arm.

So the local repository should provide the state-machine design, detection strategy, and tuning references, but not be copied blindly as the hardware-control layer.

## 3. Desktop-Wanderer Summary

Remote project:

```text
/home/orangepi/Code/Desktop-Wanderer
```

Important files:

- `src/main.py`: Python main control loop.
- `src/vision/detector.py`: Native libuvc camera wrapper exposed to Python.
- `src/yolov/process.py`: Ball and bucket detection.
- `src/yolov/models/tennis.rknn`: RK3588 tennis model.
- `src/lekiwi/lekiwi.py`: LeKiwi robot abstraction.
- `src/lekiwi/direction_control.py`: High-level movement command mapping.
- `src/robot_setup.py`: Robot construction and start-pose configuration.
- `src/move_controller.py`: Visual servo movement logic.
- `src/arm_inverse_controller.py`: Inverse-kinematics arm control.
- `config.yaml`: Runtime configuration.

Current `config.yaml`:

```yaml
port: /dev/ttyACM0
fps: 20
runtime_env: linux
log_level: INFO
hardware_mode: rk3588
control_mode: inverse
```

Observed hardware on the device:

- `/dev/ttyACM0` exists.
- UVC camera exists as USB device `0ac8:0346`.
- `/dev/video0` and `/dev/video1` exist.
- SSH session has no GUI display environment (`DISPLAY` / `WAYLAND_DISPLAY` absent).

Desktop-Wanderer hardware model:

- Arm motors:
  - `arm_shoulder_pan`
  - `arm_shoulder_lift`
  - `arm_elbow_flex`
  - `arm_wrist_flex`
  - `arm_wrist_roll`
  - `arm_gripper`
- Base motors:
  - `base_left_wheel`
  - `base_back_wheel`
  - `base_right_wheel`

Base control uses body-frame velocity commands:

```text
x.vel
y.vel
theta.vel
```

Then `LeKiwi._body_to_wheel_raw()` converts those body velocities to three base wheel velocity commands.

## 4. Architecture Decision

The new `/home/orangepi/robot` project should use Desktop-Wanderer as the hardware-control reference.

Recommended direction:

- Use Python as the first implementation language on Orange Pi.
- Reuse Desktop-Wanderer camera, RKNN model, LeKiwi robot abstraction, base control, and arm inverse-kinematics code.
- Port the local C++ repository's task state machine and behavior thresholds into the new Python project.
- Avoid directly using the local C++ `motor/` and `arm/` modules for the new robot.

Rationale:

- Desktop-Wanderer already matches the current robot shape.
- Desktop-Wanderer already talks to `/dev/ttyACM0` and Feetech STS3215 motors.
- Desktop-Wanderer already has a working RK3588 tennis `.rknn` model.
- Python allows faster staged testing for camera, vision, base, arm, and full-loop integration.

## 5. Proposed Project Structure

Recommended new structure:

```text
/home/orangepi/robot/
├── config.yaml
├── main.py
├── README.md
├── tests/
│   ├── camera_test.py
│   ├── detect_test.py
│   ├── base_test.py
│   └── arm_test.py
├── robot/
│   ├── __init__.py
│   ├── camera.py
│   ├── detector.py
│   ├── base.py
│   ├── arm.py
│   └── state_machine.py
└── thirdparty/
    └── lekiwi_or_imports/
```

This structure keeps test tools separate from the final main loop and avoids mixing experimental code into Desktop-Wanderer.

## 6. Migration Plan

### Phase 1: Camera Capture and Display

Goal: Run image acquisition in `/home/orangepi/robot` without moving the robot.

Tasks:

1. Reuse or copy Desktop-Wanderer `src/vision/detector.py`.
2. Build/load the native UVC library.
3. Open the target camera:

   ```python
   UvcCamera(640, 480, 30, vendor_id=0x0AC8, product_id=0x0346)
   ```

4. Implement `tests/camera_test.py`.
5. Save frames to disk first, for example `frame.jpg`.
6. Add optional display:
   - `cv2.imshow` only when `DISPLAY` or `WAYLAND_DISPLAY` exists.
   - Otherwise use saved images or an MJPEG HTTP preview.

Expected output:

- Camera starts reliably.
- Captured frame dimensions are known.
- Image color order is confirmed as BGR.

### Phase 2: Detection Verification

Goal: Verify tennis ball and bucket detection on live frames.

Tasks:

1. Reuse `src/yolov/process.py` and `src/yolov/models/tennis.rknn`.
2. Implement `tests/detect_test.py`.
3. Draw and save detection result images.
4. Verify:
   - Tennis detection confidence and box position.
   - Red bucket HSV detection.
   - Optional black bucket detection from Desktop-Wanderer.

Detection sources:

- Current C++ repository red-bucket HSV logic.
- Desktop-Wanderer `get_red_bucket_local()`.
- Desktop-Wanderer `get_black_bucket_local()`.

Expected output:

- `detect_test.py` can produce annotated images.
- Model runtime works in `hardware_mode: rk3588`.
- Detection latency is acceptable for 20 FPS or the chosen loop rate.

### Phase 3: Base Control Abstraction

Goal: Replace old differential motor commands with LeKiwi body-velocity commands.

Tasks:

1. Create `robot/base.py`.
2. Implement a small high-level API:

   ```python
   forward(speed_level)
   backward(speed_level)
   rotate_left(speed_level)
   rotate_right(speed_level)
   strafe_left(speed_level)
   strafe_right(speed_level)
   stop()
   ```

3. Internally use:

   ```python
   DirectionControl.get_action(...)
   robot.send_action(...)
   ```

4. Implement `tests/base_test.py` with very short, low-speed movements.

Expected output:

- The robot can stop reliably.
- Forward/backward/rotate directions are verified.
- Speed levels are safe for indoor testing.

### Phase 4: Arm Control Abstraction

Goal: Replace old ZP10D serial arm actions with Feetech/LeKiwi inverse-kinematics actions.

Tasks:

1. Reuse `arm_inverse_controller.py`.
2. Create `robot/arm.py`.
3. Provide high-level actions:

   ```python
   home()
   grab()
   release()
   lift()
   put_ball()
   ```

4. Start from Desktop-Wanderer's existing action sequences:

   - `CATCH_ACTION`
   - `PUT_ACTION`
   - `return_to_start_position()`
   - `p_control_loop()`

5. Implement `tests/arm_test.py`.

Expected output:

- Arm returns to start pose safely.
- Grab and release sequences work without chassis movement.
- Joint limits and gripper positions are confirmed.

### Phase 5: Main State Machine

Goal: Port the current C++ closed-loop behavior into the new Python hardware stack.

Recommended states:

```text
SEARCH_BALL
CHASE_BALL
PICK
FIND_BUCKET
APPROACH_BUCKET
PUT_BALL
RECOVER
```

Mapping from old C++ logic:

- `CHASE_BALL`: use YOLO tennis detection.
- Ball lost: rotate according to last-seen horizontal offset.
- Near ball: slow down and align.
- Stable target: stop base, run arm grab.
- `FIND_BUCKET`: use red/black bucket detection.
- `APPROACH_BUCKET`: align and move forward.
- Bucket close/stable: stop base, run put-ball sequence.
- Return to `SEARCH_BALL`.

The local C++ thresholds should be treated as tuning references, not final values:

- Ball area/size thresholds.
- Center dead zone.
- Search frame count.
- Reverse/stop thresholds.
- Bucket size threshold.

For the LeKiwi platform, target-box width/height logic from Desktop-Wanderer may be a better first baseline than raw area ratio.

### Phase 6: Full Integration and Tuning

Recommended integration order:

1. Camera only.
2. Camera plus detection.
3. Detection plus debug visualization.
4. Base movement single commands.
5. Arm movement single commands.
6. Detect ball but do not move.
7. Detect ball and rotate only.
8. Low-speed chase with emergency stop available.
9. Stop near ball without grabbing.
10. Add grab sequence.
11. Add bucket detection.
12. Add bucket approach.
13. Add put-ball sequence.
14. Run complete loop.

## 7. Safety and Operational Notes

- Always start with low speed.
- Make `stop()` easy to call and use it in `finally`.
- Keep base velocity zero during arm motion unless explicitly needed.
- Avoid enabling old `my-car.service` while developing, otherwise it may compete for `/dev/ttyACM0` or the camera.
- Since SSH has no display environment, prefer saved debug images or a web preview for headless debugging.
- Run base and arm tests with the robot lifted or in a clear area first.

## 8. Immediate Next Steps

1. Populate `/home/orangepi/robot` with a minimal Python project.
2. Add `config.yaml`.
3. Add `tests/camera_test.py`.
4. Run camera capture and save one frame.
5. Add `tests/detect_test.py`.
6. Verify tennis RKNN inference on live camera frames.
7. Only after vision is stable, add base and arm tests.

## 9. Current Conclusion

The correct migration strategy is:

- Use current local C++ repository as the behavior and algorithm reference.
- Use Desktop-Wanderer as the hardware-control reference.
- Build the new `/home/orangepi/robot` project in small testable layers.

This avoids fighting the old differential-drive and ZP10D assumptions while preserving the existing ball-catching and bucket-deposit closed-loop design.
