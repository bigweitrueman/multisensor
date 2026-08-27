#!/usr/bin/env python3
import argparse
import datetime as dt
import os
import signal
import struct
import sys
import termios
import time


PWM_PERIOD_NS = 1_000_000_000
DEFAULT_DUTY_NS = 10_000_000
TRIGGER_PERIOD_NS = 100_000_000
TRIGGER_DUTY_NS = 5_000_000


class StopFlag:
    def __init__(self):
        self.stop = False

    def handler(self, _signum, _frame):
        self.stop = True


def write_file(path, value):
    with open(path, "w", encoding="ascii") as f:
        f.write(str(value))


def read_file(path):
    with open(path, "r", encoding="ascii") as f:
        return f.read().strip()


def ensure_pwm(chip, channel):
    pwm_dir = os.path.join(chip, f"pwm{channel}")
    export_path = os.path.join(chip, "export")

    if not os.path.isdir(pwm_dir):
        write_file(export_path, channel)
        deadline = time.monotonic() + 2.0
        while not os.path.isdir(pwm_dir):
            if time.monotonic() > deadline:
                raise RuntimeError(f"PWM channel did not appear: {pwm_dir}")
            time.sleep(0.02)
    return pwm_dir


def configure_pwm(pwm_dir, period_ns, duty_ns, polarity, keep_running=False):
    enable_path = os.path.join(pwm_dir, "enable")
    if not keep_running:
        try:
            write_file(enable_path, 0)
        except OSError:
            pass

    write_file(os.path.join(pwm_dir, "period"), period_ns)
    write_file(os.path.join(pwm_dir, "duty_cycle"), duty_ns)
    write_file(os.path.join(pwm_dir, "polarity"), polarity)
    return enable_path


def setup_pwm(chip, channel, period_ns, duty_ns, polarity, keep_running=False):
    pwm_dir = ensure_pwm(chip, channel)
    enable_path = configure_pwm(pwm_dir, period_ns, duty_ns, polarity, keep_running)
    write_file(enable_path, 1)
    return pwm_dir


def prepare_pwm(chip, channel, period_ns, duty_ns, polarity):
    """Configure a sysfs PWM but leave it disabled for an epoch-bound start."""
    pwm_dir = ensure_pwm(chip, channel)
    enable_path = configure_pwm(pwm_dir, period_ns, duty_ns, polarity)
    return pwm_dir, enable_path


def setup_pwm_group(configs, keep_running=False):
    prepared = []
    for name, chip, channel, period_ns, duty_ns, polarity in configs:
        pwm_dir = ensure_pwm(chip, channel)
        enable_path = configure_pwm(pwm_dir, period_ns, duty_ns, polarity, keep_running)
        prepared.append((name, pwm_dir, enable_path))

    # Start all outputs back-to-back. This is not hardware-synchronous, but it
    # gives both PWM generators the tightest practical software start point.
    epoch_wall_ns = time.time_ns()
    epoch_mono_ns = time.monotonic_ns()
    epoch_mono = epoch_mono_ns / 1_000_000_000
    for _name, _pwm_dir, enable_path in prepared:
        write_file(enable_path, 1)

    outputs = [(name, pwm_dir) for name, pwm_dir, _enable_path in prepared]
    return outputs, epoch_wall_ns, epoch_mono_ns, epoch_mono


def drain_trigger_device(fd):
    while True:
        try:
            data = os.read(fd, 16)
        except BlockingIOError:
            return
        except OSError:
            return
        if len(data) != 16:
            return


def enable_imx586_trigger_bound_epoch(trigger_dev, trigger_enable, pps_phase_ns):
    fd = os.open(trigger_dev, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        write_file(trigger_enable, 0)
        drain_trigger_device(fd)
        write_file(trigger_enable, 1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                data = os.read(fd, 16)
            except BlockingIOError:
                time.sleep(0.001)
                continue
            if len(data) != 16:
                time.sleep(0.001)
                continue
            timestamp_ns, counter = struct.unpack("<Qq", data)
            if counter == -1:
                pps_epoch_ns = timestamp_ns + pps_phase_ns
                return timestamp_ns, pps_epoch_ns
        raise RuntimeError("timed out waiting for imx586 trigger base event")
    finally:
        os.close(fd)


def write_state_file(path, values):
    if not path:
        return

    temp_path = f"{path}.tmp.{os.getpid()}"
    with open(temp_path, "w", encoding="ascii") as state:
        for key, value in values.items():
            state.write(f"{key}={value}\n")
        state.flush()
        os.fsync(state.fileno())
    os.replace(temp_path, path)


def configure_serial(path, baud):
    baud_map = {
        4800: termios.B4800,
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baud not in baud_map:
        raise ValueError(f"unsupported baud: {baud}")

    fd = os.open(path, os.O_WRONLY | os.O_NOCTTY | os.O_SYNC)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_map[baud]
    attrs[5] = baud_map[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def nmea_checksum(payload):
    value = 0
    for ch in payload:
        value ^= ord(ch)
    return f"{value:02X}"


def build_rmc(sentence_prefix, utc_time, lat, ns, lon, ew, status, speed, course, mode):
    hhmmss = utc_time.strftime("%H%M%S")
    ddmmyy = utc_time.strftime("%d%m%y")
    payload = (
        f"{sentence_prefix}RMC,{hhmmss}.00,{status},{lat},{ns},"
        f"{lon},{ew},{speed},{course},{ddmmyy},,,{mode}"
    )
    return f"${payload}*{nmea_checksum(payload)}\r\n"


def parse_start(value):
    if value.lower() == "default":
        return dt.datetime(2026, 1, 1, 0, 0, 0)
    try:
        return dt.datetime.strptime(value, "%Y-%m-%dT%H:%M:%S")
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use YYYY-MM-DDTHH:MM:SS or default") from exc


def sleep_until(target_monotonic, stop_flag):
    while not stop_flag.stop:
        remain = target_monotonic - time.monotonic()
        if remain <= 0:
            return
        time.sleep(min(remain, 0.05))


def main():
    parser = argparse.ArgumentParser(
        description="Generate hardware PPS and synthetic UTC RMC for WLR-722 GPS sync."
    )
    parser.add_argument("--pwm-chip", default="/sys/class/pwm/pwmchip0")
    parser.add_argument("--pwm-channel", type=int, default=0)
    parser.add_argument("--period-ns", type=int, default=PWM_PERIOD_NS)
    parser.add_argument("--duty-ns", type=int, default=DEFAULT_DUTY_NS)
    parser.add_argument("--polarity", choices=("normal", "inversed"), default="normal")
    parser.add_argument("--enable-trigger", action="store_true")
    parser.add_argument("--trigger-pwm-chip", default="/sys/class/pwm/pwmchip1")
    parser.add_argument("--trigger-pwm-channel", type=int, default=0)
    parser.add_argument("--trigger-period-ns", type=int, default=TRIGGER_PERIOD_NS)
    parser.add_argument("--trigger-duty-ns", type=int, default=TRIGGER_DUTY_NS)
    parser.add_argument("--trigger-polarity", choices=("normal", "inversed"), default="normal")
    parser.add_argument("--keep-pwm-running", action="store_true")
    parser.add_argument("--skip-pwm", action="store_true")
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--talker", choices=("GN", "GP"), default="GN")
    parser.add_argument("--start", type=parse_start, default=parse_start("default"))
    parser.add_argument("--send-offset-ms", type=float, default=120.0)
    parser.add_argument("--lat", default="0000.0000")
    parser.add_argument("--ns", choices=("N", "S"), default="N")
    parser.add_argument("--lon", default="00000.0000")
    parser.add_argument("--ew", choices=("E", "W"), default="E")
    parser.add_argument("--status", choices=("A", "V"), default="A")
    parser.add_argument("--speed", default="0.0")
    parser.add_argument("--course", default="0.0")
    parser.add_argument("--mode", default="A")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--state-file", default="/tmp/vanjee_sync_state")
    parser.add_argument("--bind-imx586-trigger", action="store_true")
    parser.add_argument("--imx586-trigger-dev", default="/dev/imx586_trigger")
    parser.add_argument("--imx586-trigger-enable", default="/sys/kernel/imx586_trigger/enable")
    parser.add_argument("--imx586-pps-phase-ns", type=int, default=25778000)
    parser.add_argument(
        "--pps-pwm-chip", default="",
        help="Optional sysfs PWM chip for PPS, enabled at the bound epoch.",
    )
    parser.add_argument("--pps-pwm-channel", type=int, default=0)
    args = parser.parse_args()

    if args.send_offset_ms < 0 or args.send_offset_ms >= 1000:
        raise SystemExit("--send-offset-ms must be in [0, 1000)")

    stop_flag = StopFlag()
    signal.signal(signal.SIGINT, stop_flag.handler)
    signal.signal(signal.SIGTERM, stop_flag.handler)

    epoch_wall_ns = time.time_ns()
    epoch_mono_ns = time.monotonic_ns()
    epoch_mono = epoch_mono_ns / 1_000_000_000
    outputs = []
    pps_pwm_enable_path = None

    if args.pps_pwm_chip:
        pps_pwm_dir, pps_pwm_enable_path = prepare_pwm(
            args.pps_pwm_chip, args.pps_pwm_channel,
            args.period_ns, args.duty_ns, args.polarity)
        print(
            "PPS PWM prepared:",
            f"dir={pps_pwm_dir}",
            f"period={read_file(os.path.join(pps_pwm_dir, 'period'))}",
            f"duty={read_file(os.path.join(pps_pwm_dir, 'duty_cycle'))}",
            flush=True,
        )
    elif args.skip_pwm:
        print("PWM skipped.", flush=True)
    else:
        pwm_configs = [
            (
                "pps",
                args.pwm_chip,
                args.pwm_channel,
                args.period_ns,
                args.duty_ns,
                args.polarity,
            )
        ]
        if args.enable_trigger:
            pwm_configs.append(
                (
                    "trigger",
                    args.trigger_pwm_chip,
                    args.trigger_pwm_channel,
                    args.trigger_period_ns,
                    args.trigger_duty_ns,
                    args.trigger_polarity,
                )
            )

        outputs, epoch_wall_ns, epoch_mono_ns, epoch_mono = setup_pwm_group(
            pwm_configs, args.keep_pwm_running
        )
        for name, pwm_dir in outputs:
            print(
                "PWM enabled:",
                f"name={name}",
                f"dir={pwm_dir}",
                f"period={read_file(os.path.join(pwm_dir, 'period'))}",
                f"duty={read_file(os.path.join(pwm_dir, 'duty_cycle'))}",
                f"polarity={read_file(os.path.join(pwm_dir, 'polarity'))}",
                flush=True,
            )

    serial_fd = None
    if not args.dry_run:
        serial_fd = configure_serial(args.serial, args.baud)
        print(f"Serial opened: {args.serial} baud={args.baud}", flush=True)
    else:
        print("Dry-run mode: RMC sentences are printed, not sent.", flush=True)

    imx586_trigger_base_ns = 0
    imx586_pps_epoch_ns = 0
    if args.bind_imx586_trigger:
        imx586_trigger_base_ns, imx586_pps_epoch_ns = enable_imx586_trigger_bound_epoch(
            args.imx586_trigger_dev,
            args.imx586_trigger_enable,
            args.imx586_pps_phase_ns,
        )
        epoch_mono_ns = imx586_pps_epoch_ns
        epoch_mono = epoch_mono_ns / 1_000_000_000
        epoch_wall_ns = time.time_ns() + (epoch_mono_ns - time.monotonic_ns())
        print(
            "IMX586 trigger bound:",
            f"camera_base_ns={imx586_trigger_base_ns}",
            f"pps_epoch_ns={imx586_pps_epoch_ns}",
            f"pps_phase_ns={args.imx586_pps_phase_ns}",
            flush=True,
        )

    if pps_pwm_enable_path is not None:
        # GPIO54 trigger base is the master epoch. Start PWM13 PPS at the
        # configured phase so the radar sees the same epoch as the camera.
        sleep_until(epoch_mono, stop_flag)
        if not stop_flag.stop:
            write_file(pps_pwm_enable_path, 1)
            print("PPS PWM enabled at bound epoch.", flush=True)

    write_state_file(
        args.state_file,
        {
            "version": 1,
            "pid": os.getpid(),
            "epoch_wall_ns": epoch_wall_ns,
            "epoch_mono_ns": epoch_mono_ns,
            "period_ns": args.period_ns,
            "trigger_enabled": int((args.enable_trigger and not args.skip_pwm) or args.bind_imx586_trigger),
            "trigger_period_ns": args.trigger_period_ns,
            "trigger_duty_ns": args.trigger_duty_ns,
            "trigger_polarity": args.trigger_polarity,
            "imx586_bound": int(args.bind_imx586_trigger),
            "imx586_trigger_base_ns": imx586_trigger_base_ns,
            "imx586_pps_epoch_ns": imx586_pps_epoch_ns,
            "imx586_pps_phase_ns": args.imx586_pps_phase_ns,
            "start_local": args.start.isoformat(),
            "serial": args.serial,
            "baud": args.baud,
        },
    )
    print(f"Sync state written: {args.state_file}", flush=True)

    # PWM enable time is the common epoch for PPS, camera trigger, and RMC.
    pseudo_utc = args.start
    tick = 0
    offset_sec = args.send_offset_ms / 1000.0

    try:
        while not stop_flag.stop:
            send_at = epoch_mono + tick + offset_sec
            sleep_until(send_at, stop_flag)
            if stop_flag.stop:
                break

            sentence = build_rmc(
                args.talker,
                pseudo_utc + dt.timedelta(seconds=tick),
                args.lat,
                args.ns,
                args.lon,
                args.ew,
                args.status,
                args.speed,
                args.course,
                args.mode,
            )
            if serial_fd is None:
                if args.print_every > 0 and tick % args.print_every == 0:
                    print(f"{tick:06d} {sentence}", end="", flush=True)
            else:
                os.write(serial_fd, sentence.encode("ascii"))
                if args.print_every > 0 and tick % args.print_every == 0:
                    print(f"{tick:06d} sent {sentence}", end="", flush=True)

            tick += 1
            next_tick = epoch_mono + tick
            if time.monotonic() > next_tick + 0.5:
                print("warning: sender is more than 500 ms late", file=sys.stderr, flush=True)
    finally:
        if serial_fd is not None:
            os.close(serial_fd)
        print("stopped", flush=True)


if __name__ == "__main__":
    main()
