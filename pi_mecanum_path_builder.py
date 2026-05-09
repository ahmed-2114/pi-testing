#!/usr/bin/env python3

import time
from dataclasses import dataclass

from pi_mecanum_ir_stepper_control import (
    ACK_TIMEOUT_S,
    BAUD,
    MOVE_DONE_TIMEOUT_S,
    PORT,
    EspPiControlLink,
    MecanumIrStepperSupervisor,
    initialize_robot,
    print_done,
    request_status,
    require_ok_ack,
    telemetry_summary,
)


PATH_HEADING_DEG = 0.0
IR_BUZZER_ON_S = 1.0
IR_RECHECK_DELAY_S = 3.0
MIN_RESUME_DISTANCE_CM = 0.5

# ESP MOVE angles are measured from forward. This robot's physical strafe wiring
# matches the older commander: left is positive strafe, right is negative strafe.
DIRECTION_ORDER = ("F", "R", "L", "B", "BR", "BL", "FR", "FL")

DIRECTION_ANGLES_DEG = {
    "F": 0.0,
    "FR": -45.0,
    "R": -90.0,
    "BR": -135.0,
    "B": 180.0,
    "BL": 135.0,
    "L": 90.0,
    "FL": 45.0,
}

STEPPER_DIRECTIONS = {
    "U": -1,
    "UP": -1,
    "D": 1,
    "DOWN": 1,
}


@dataclass(frozen=True)
class Waypoint:
    direction: str
    angle_deg: float
    distance_cm: float
    speed_rpm: float


def prompt_waypoint_count() -> int:
    while True:
        raw = input("Number of waypoints: ").strip()
        try:
            count = int(raw)
        except ValueError:
            print("input error | enter a whole number greater than 0")
            continue

        if count <= 0:
            print("input error | waypoint count must be greater than 0")
            continue

        return count


def parse_waypoint(raw: str) -> Waypoint:
    parts = raw.split()
    if len(parts) != 3:
        raise ValueError("use: <direction> <distance_cm> <rpm>")

    direction = parts[0].upper()
    if direction not in DIRECTION_ANGLES_DEG:
        allowed = " ".join(DIRECTION_ORDER)
        raise ValueError(f"direction must be one of: {allowed}")

    distance_cm = float(parts[1])
    speed_rpm = float(parts[2])

    if distance_cm <= 0.0:
        raise ValueError("distance must be greater than 0 cm")
    if speed_rpm <= 0.0:
        raise ValueError("rpm must be greater than 0")

    return Waypoint(
        direction=direction,
        angle_deg=DIRECTION_ANGLES_DEG[direction],
        distance_cm=distance_cm,
        speed_rpm=speed_rpm,
    )


def prompt_waypoints(count: int) -> list[Waypoint]:
    waypoints: list[Waypoint] = []
    print(f"Directions: {' '.join(DIRECTION_ORDER)}")
    print("Waypoint format: <direction> <distance_cm> <rpm>")

    for index in range(1, count + 1):
        while True:
            raw = input(f"Waypoint {index}/{count}: ").strip()
            try:
                waypoint = parse_waypoint(raw)
            except ValueError as exc:
                print(f"input error | {exc}")
                continue

            waypoints.append(waypoint)
            break

    return waypoints


def prompt_stepper_direction() -> int:
    while True:
        raw = input("Final stepper direction (up/down or u/d): ").strip().upper()
        if raw in STEPPER_DIRECTIONS:
            return STEPPER_DIRECTIONS[raw]

        print("input error | enter up, down, u, or d")


def print_path_summary(waypoints: list[Waypoint], stepper_direction: int) -> None:
    print("path summary")
    for index, waypoint in enumerate(waypoints, start=1):
        print(
            f"  {index}. {waypoint.direction} | "
            f"angle={waypoint.angle_deg:.1f}deg "
            f"distance={waypoint.distance_cm:.2f}cm "
            f"rpm={waypoint.speed_rpm:.2f}"
        )

    stepper_text = "up" if stepper_direction < 0 else "down"
    print(f"  final stepper: {stepper_text} ({stepper_direction})")


def wait_for_ir_clear(supervisor: MecanumIrStepperSupervisor) -> None:
    print("ir pause | obstacle detected, waiting until sensors are clear")

    while True:
        active = supervisor.ir_monitor.active_sensors()
        active_text = active if active else "none"
        print(f"ir pause | active sensors={active_text}, buzzer on for {IR_BUZZER_ON_S:.0f}s")
        supervisor.indicators.set_buzzer(True)
        time.sleep(IR_BUZZER_ON_S)
        supervisor.indicators.set_buzzer(False)

        print(f"ir pause | rechecking in {IR_RECHECK_DELAY_S:.0f}s")
        time.sleep(IR_RECHECK_DELAY_S)

        if not supervisor.ir_monitor.active_sensors():
            supervisor.clear_latch()
            print("ir pause | clear, resuming path")
            return


def remaining_after_stop(waypoint: Waypoint, current_remaining_cm: float, done: dict) -> float:
    try:
        progress_cm = float(done.get("progressCm", 0.0))
    except (TypeError, ValueError):
        progress_cm = 0.0

    progress_cm = max(0.0, progress_cm)
    remaining_cm = max(0.0, current_remaining_cm - progress_cm)
    print(
        f"path pause | {waypoint.direction} progress={progress_cm:.2f}cm "
        f"remaining={remaining_cm:.2f}cm"
    )
    return remaining_cm


def execute_path_segment(
    link: EspPiControlLink,
    supervisor: MecanumIrStepperSupervisor,
    waypoint: Waypoint,
    index: int,
    total: int,
) -> None:
    print(
        f"path {index}/{total} | "
        f"{waypoint.direction} {waypoint.distance_cm:.2f}cm @ {waypoint.speed_rpm:.2f}rpm"
    )

    remaining_cm = waypoint.distance_cm
    attempt = 1

    while remaining_cm > MIN_RESUME_DISTANCE_CM:
        if not supervisor.motion_allowed():
            wait_for_ir_clear(supervisor)

        line = (
            f"MOVE angle={waypoint.angle_deg} "
            f"dist={remaining_cm} "
            f"speed={waypoint.speed_rpm} "
            f"heading={PATH_HEADING_DEG}"
        )

        if attempt > 1:
            print(
                f"path {index}/{total} | resuming {waypoint.direction} "
                f"remaining={remaining_cm:.2f}cm"
            )
        print(f"send | {line}")

        supervisor.set_mecanum_active(True)
        try:
            seq = link.send_command(line)
            ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
            require_ok_ack(ack)
            print(f"ack  | {ack.get('message')}")

            done = link.wait_for_done(seq, timeout=MOVE_DONE_TIMEOUT_S)
            print_done(done)
        finally:
            supervisor.set_mecanum_active(False)

        if done.get("result") == "completed":
            return

        if done.get("result") == "stopped" and supervisor.ir_stop_latched.is_set():
            remaining_cm = remaining_after_stop(waypoint, remaining_cm, done)
            wait_for_ir_clear(supervisor)
            attempt += 1
            continue

        raise RuntimeError(
            f"path stopped at waypoint {index}/{total}: result={done.get('result')}"
        )

    if supervisor.ir_stop_latched.is_set():
        wait_for_ir_clear(supervisor)
    print(f"path {index}/{total} | remaining distance below resume threshold, continuing")


def execute_path(
    link: EspPiControlLink,
    supervisor: MecanumIrStepperSupervisor,
    waypoints: list[Waypoint],
    stepper_direction: int,
) -> None:
    supervisor.stepper.set_direction(stepper_direction)

    for index, waypoint in enumerate(waypoints, start=1):
        execute_path_segment(link, supervisor, waypoint, index, len(waypoints))

    supervisor.ensure_motion_allowed()
    print("path complete | running final stepper")
    supervisor.run_stepper_after_move()


def main() -> None:
    count = prompt_waypoint_count()
    waypoints = prompt_waypoints(count)
    stepper_direction = prompt_stepper_direction()
    print_path_summary(waypoints, stepper_direction)

    link: EspPiControlLink | None = None
    supervisor: MecanumIrStepperSupervisor | None = None

    try:
        link = EspPiControlLink(PORT, BAUD)
        print(f"opened {PORT} @ {BAUD}")
        link.drain(1.2)
        initialize_robot(link)
        data = request_status(link)
        print(telemetry_summary(data))

        supervisor = MecanumIrStepperSupervisor(link)
        print(supervisor.ir_monitor.status_text())
        print(supervisor.stepper.status_text())
        print("IR note | startup baseline was captured when the script started, so begin with the path clear.")

        execute_path(link, supervisor, waypoints, stepper_direction)
    except KeyboardInterrupt:
        print("\ninterrupted | sending STOP")
        if link is not None:
            try:
                link.send_command("STOP")
            except Exception as exc:
                print(f"stop warning | {exc}")
    except (RuntimeError, TimeoutError) as exc:
        print(f"error | {exc}")
        if link is not None:
            try:
                link.send_command("STOP")
            except Exception as stop_exc:
                print(f"stop warning | {stop_exc}")
    finally:
        if supervisor is not None:
            supervisor.close()
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
