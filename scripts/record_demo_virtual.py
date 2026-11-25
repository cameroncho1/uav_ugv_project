#!/usr/bin/env python3
"""Linux worker for record_demo_video.cjs; owns one isolated display and chapter.

Only the existing dashboard rehearsal issues controls. This worker records pixels,
bookkeeps process ownership, and writes timing/cleanup evidence to a fresh directory.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import uuid
from urllib.error import URLError
from urllib.request import urlopen


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def tagged_processes(tag):
    """Include adopted/separate-session PX4 descendants, using an exact private tag."""
    marker = f"UAV_RECORDING_SESSION={tag}".encode()
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            if marker in (entry / "environ").read_bytes().split(b"\0"):
                # Zombies have no environment and cannot be signalled into exiting.
                result.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return result


def require_free_port(port, kind):
    with socket.socket(socket.AF_INET, kind) as probe:
        if kind == socket.SOCK_STREAM:
            # A completed owned server may leave TIME_WAIT connections. This
            # still refuses an active listener, matching HTTPServer rebinding.
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("0.0.0.0", port))


def run(args):
    root, output = args.root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    tag = f"demo_video_{uuid.uuid4().hex}"
    env = dict(os.environ, UAV_RECORDING_SESSION=tag, GZ_PARTITION=tag,
               ROS_DOMAIN_ID=str(args.domain), QT_QPA_PLATFORM="xcb",
               ROS_LOCALHOST_ONLY="1",
               LP_NUM_THREADS="2",
               QT_X11_NO_MITSHM="1")
    env.pop("WAYLAND_DISPLAY", None)
    env.pop("DISPLAY", None)
    # Mesa can accelerate Xvfb through WSL's D3D12 device. Let it select the
    # available driver; forcibly using llvmpipe can starve sensor rendering.
    env.pop("LIBGL_ALWAYS_SOFTWARE", None)
    env.pop("GALLIUM_DRIVER", None)
    children, streams = [], []
    recorder = None
    metadata = dict(chapter=args.chapter, session=tag, ros_domain_id=args.domain,
                    dashboard_port=args.port,
                    wayland_display=None, success=False)

    def start(command, name, **kwargs):
        log = (output / f"{name}.log").open("w")
        streams.append(log)
        process = subprocess.Popen(command, cwd=root, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True,
                                   **kwargs)
        children.append(process)
        return process

    def check_stop():
        if (output / "stop").exists():
            raise InterruptedError("Controller requested cleanup")

    def interrupted(signum, _frame):
        raise InterruptedError(f"Received signal {signum}")

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        for command in ("Xvfb", "openbox", "xdotool", "ffmpeg"):
            if not shutil.which(command):
                raise RuntimeError(f"Missing Linux dependency: {command}")
        if not (root / "ros2_ws/install/setup.bash").exists():
            raise RuntimeError("A built Linux workspace is required")
        source_root = Path(__file__).resolve().parents[1]
        metadata["source_hashes"] = {}
        for relative in ("scripts/run_demo.sh", "scripts/run_ml_demo.sh",
                         "scripts/rehearse_demo.py", "scripts/rehearse_ml_demo.py",
                         "scripts/world_goal_demo.sh", "simulation/gui/demo.config"):
            intended = (source_root / relative).read_text().encode()
            actual = (root / relative).read_text().encode()
            if intended != actual:
                raise RuntimeError(f"Linux source is stale: {relative}; sync it with LF line endings")
            metadata["source_hashes"][relative] = hashlib.sha256(actual).hexdigest()
        if args.chapter == "ml":
            # A closer presentation camera keeps the complete -4 m to +4 m drive
            # in view. This changes only GUI framing; the world stays identical.
            gui = (root / "simulation/gui/demo.config").read_text().replace(
                "2 -14 16 0 0.85 1.5708", "0 -8 9 0 0.85 1.5708")
            gui_path = output / "ml-view.config"
            gui_path.write_text(gui)
            env["GZ_GUI_CONFIG"] = str(gui_path)
            metadata["gui_camera_pose"] = "0 -8 9 0 0.85 1.5708"
        # The launch has a fixed XRCE port. Never attach to or replace another run.
        require_free_port(args.port, socket.SOCK_STREAM)
        require_free_port(8888, socket.SOCK_DGRAM)
        domain_marker = f"ROS_DOMAIN_ID={args.domain}".encode()
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal() or int(entry.name) == os.getpid():
                continue
            try:
                if domain_marker in (entry / "environ").read_bytes().split(b"\0"):
                    raise RuntimeError(f"ROS domain {args.domain} is already in use by PID {entry.name}")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                pass
        # WSLg's pathname socket directory can be a read-only mount. Keep its
        # sockets untouched and use Linux's separate abstract local transport.
        # Choose an explicit private number: automatic allocation could pick
        # abstract :0 while WSLg owns only pathname :0.
        display_number = None
        for candidate in range(100, 300):
            if Path(f"/tmp/.X11-unix/X{candidate}").exists() or Path(f"/tmp/.X{candidate}-lock").exists():
                continue
            with socket.socket(socket.AF_UNIX) as probe:
                try:
                    probe.bind(f"\0/tmp/.X11-unix/X{candidate}")
                except OSError:
                    continue
            display_number = candidate
            break
        if display_number is None:
            raise RuntimeError("No unused private X display available")
        read_fd, write_fd = os.pipe()
        try:
            xserver = start(["Xvfb", f":{display_number}", "-displayfd", str(write_fd),
                             "-screen", "0", "1280x960x24", "-nolisten", "tcp",
                             "-nolisten", "unix", "-pn", "-noreset"],
                            "xvfb", pass_fds=(write_fd,))
            os.close(write_fd)
            write_fd = None
            # Xvfb writes the allocated number only after accepting connections.
            import select
            if not select.select([read_fd], [], [], 20)[0]:
                raise RuntimeError("Xvfb did not allocate a display")
            number = os.read(read_fd, 32).decode().strip()
        finally:
            os.close(read_fd)
            if write_fd is not None:
                os.close(write_fd)
        if not number.isdecimal() or int(number) == 0:
            raise RuntimeError(f"Refusing non-private display: {number!r}")
        env["DISPLAY"] = f":{number}"
        metadata.update(display=env["DISPLAY"], xvfb_pid=xserver.pid,
                        capture_geometry="1280x960", capture_fps=12)
        if shutil.which("glxinfo"):
            renderer = subprocess.run(["glxinfo", "-B"], env=env,
                                      capture_output=True, text=True, timeout=15)
            (output / "renderer.log").write_text(renderer.stdout + renderer.stderr)
            metadata["renderer"] = next((line.split(": ", 1)[1] for line in
                renderer.stdout.splitlines() if line.startswith("OpenGL renderer string:")), "unknown")
            metadata["software_rendering"] = "llvmpipe" in metadata["renderer"].lower()
        start(["openbox", "--sm-disable"], "openbox")
        launcher = "run_demo.sh" if args.chapter == "navigation" else "run_ml_demo.sh"
        launch = start(["bash", str(root / "scripts" / launcher),
                        f"dashboard_port:={args.port}"], "simulation")
        base = f"http://127.0.0.1:{args.port}"
        deadline = time.monotonic() + args.startup_timeout
        window = None
        while time.monotonic() < deadline:
            check_stop()
            if launch.poll() is not None:
                raise RuntimeError("Simulation exited during startup; inspect simulation.log")
            found = subprocess.run(["xdotool", "search", "--onlyvisible", "--name", "Gazebo"],
                                   env=env, capture_output=True, text=True)
            try:
                with urlopen(base + "/state", timeout=1) as response:
                    json.load(response)
                if found.returncode == 0:
                    window = found.stdout.splitlines()[-1]
                    break
            except (URLError, TimeoutError):
                pass
            time.sleep(0.25)
        if window is None:
            raise RuntimeError("Private Gazebo window/dashboard startup deadline")
        subprocess.run(["xdotool", "windowsize", window, "1276", "930",
                        "windowmove", window, "0", "0"], env=env, check=True)
        metadata["gazebo_window_id"] = window
        timeline = (output / "source_timeline.jsonl").open("w")
        streams.append(timeline)
        layer_sequence = -1
        last_sim_time, last_progress = None, time.monotonic()

        def snapshot():
            nonlocal layer_sequence, last_sim_time, last_progress
            query = "&".join(f"{key}=999999999" for key in
                             ("truth", "fused", "baseline", "series", "ml"))
            query += f"&layers={layer_sequence}"
            with urlopen(base + "/state?" + query, timeout=3) as response:
                state = json.load(response)
            layer_sequence = state["layers_seq"]
            if last_sim_time is None or state["t"] > last_sim_time:
                last_sim_time, last_progress = state["t"], time.monotonic()
            elif time.monotonic() - last_progress > 45:
                raise RuntimeError("Dashboard simulation clock stopped advancing for 45 wall seconds")
            timeline.write(json.dumps(dict(utc_ms=time.time() * 1000,
                **{key: state.get(key) for key in ("t", "mode", "truth", "flags")})) + "\n")
            timeline.flush()

        snapshot()
        write_json(output / "ready.json", dict(**metadata, ready_utc_ms=time.time() * 1000))
        deadline = time.monotonic() + 90
        while not (output / "browser-ready.json").exists():
            check_stop()
            if time.monotonic() > deadline:
                raise RuntimeError("Headless dashboard controller did not acknowledge readiness")
            time.sleep(0.1)
        metadata["gazebo_capture_start_utc_ms"] = time.time() * 1000
        recorder = start(["ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
                          "-f", "x11grab", "-draw_mouse", "0", "-framerate", "12",
                          "-video_size", "1280x960", "-i", env["DISPLAY"] + ".0+0,0",
                          "-an", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "20",
                          "-threads", "2", "-pix_fmt", "yuv420p", str(output / "gazebo.mkv")],
                         "capture")
        time.sleep(0.5)
        if recorder.poll() is not None:
            raise RuntimeError("Private-display recorder failed; inspect capture.log")
        rehearsal = "rehearse_demo.py" if args.chapter == "navigation" else "rehearse_ml_demo.py"
        driver = start(["python3", str(root / "scripts" / rehearsal), "--url", base,
                        "--timeout", str(args.trial_timeout), "--output", str(output / "evidence")],
                       "rehearsal")
        deadline = time.monotonic() + args.trial_timeout + 15
        next_snapshot = time.monotonic()
        while driver.poll() is None:
            check_stop()
            if time.monotonic() > deadline:
                raise RuntimeError("Rehearsal worker exceeded deadline")
            if launch.poll() is not None or recorder.poll() is not None:
                raise RuntimeError("Simulation or recording exited during rehearsal")
            if time.monotonic() >= next_snapshot:
                snapshot()
                next_snapshot = time.monotonic() + 0.5
            time.sleep(0.2)
        if driver.returncode:
            raise RuntimeError("Rehearsal checks failed; inspect evidence/summary.json")
        metadata["rehearsal_finished_utc_ms"] = time.time() * 1000
        write_json(output / "rehearsal-finished.json",
                   dict(utc_ms=metadata["rehearsal_finished_utc_ms"]))
        # A real, still-live final shot: no frozen/reconstructed dashboard values.
        final_until = time.monotonic() + (5 if args.chapter == "ml" else 2)
        while time.monotonic() < final_until:
            check_stop()
            snapshot()
            time.sleep(0.1)
        metadata["capture_end_utc_ms"] = time.time() * 1000
        metadata["success"] = True
    except BaseException as error:
        metadata["failure"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        # First close the encoder normally so even failed runs retain readable footage.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait()
        owned = tagged_processes(tag)
        metadata["cleanup_signalled_pids"] = owned
        for pid in owned:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 8
        while tagged_processes(tag) and time.monotonic() < deadline:
            for child in children:
                child.poll()
            time.sleep(0.1)
        for pid in tagged_processes(tag):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for child in children:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
        time.sleep(0.2)
        metadata["cleanup_remaining_pids"] = tagged_processes(tag)
        metadata["cleanup_completed_utc_ms"] = time.time() * 1000
        for stream in streams:
            stream.close()
        write_json(output / "worker.json", metadata)
        if metadata["cleanup_remaining_pids"]:
            raise RuntimeError("Owned processes remain after cleanup")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--chapter", choices=("navigation", "ml"), required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--startup-timeout", type=float, default=180)
    parser.add_argument("--trial-timeout", type=float, default=900)
    run(parser.parse_args())
