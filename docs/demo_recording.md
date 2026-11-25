# Private-display demo recording

[Demo script](demo.md) | [Validation](demo_validation.md)

- `scripts/record_demo_video.cjs` records the navigation/localization chapter, stops that simulation, and records the ML chapter in a fresh simulation. Existing rehearsal drivers operate the real dashboard controls. The ML chapter ends after fault clearing, resumed driving and goal arrival; the optional UAV-timeout extension is excluded.
- Gazebo renders on a private Linux Xvfb display numbered 100 or higher. `QT_QPA_PLATFORM=xcb`, an unset `WAYLAND_DISPLAY`, and an explicit private `DISPLAY` keep its GUI off the visible WSLg desktop. Xvfb disables TCP and pathname listeners; Linux abstract local X sockets work even when WSLg mounts `/tmp/.X11-unix` read-only. The workflow does not change that mount.
- Windows headless Playwright Chromium records only its dashboard page. Linux ffmpeg records only the owned virtual display. Neither capture reads the Windows desktop, WSLg `:0`, unrelated applications, microphones or system audio. Normal Windows applications remain usable.

## Dependencies and source

- Use the built Linux workspace from [WSL reproduction](demo_validation.md#wsl-reproduction). ROS 2 Humble, Gazebo Harmonic, PX4 and MicroXRCEAgent run in Linux. Node, Playwright Chromium, ffmpeg and ffprobe run in Windows. No Linux Node installation is required.
- Install the small Linux capture dependencies from PowerShell:

  ```powershell
  wsl -d Ubuntu-22.04 -e sudo apt-get install -y xvfb ffmpeg openbox xdotool
  ```

- Verify Windows dependencies from the repository directory:

  ```powershell
  node -e "console.log(require.resolve('playwright')); console.log(require('playwright').chromium.executablePath())"
  ffmpeg -version
  ffprobe -version
  ```

- If Playwright is missing, install it in the Windows user directory with `npm install --prefix "$env:USERPROFILE" playwright`, followed by `npx --prefix "$env:USERPROFILE" playwright install chromium`. The repository needs no npm project.
- Keep the built Linux copy current. Before starting any processes, the worker compares normalized hashes of the launchers, rehearsal drivers, world helper and GUI configuration against this checkout. A mismatch reports the stale file. Sync that file to the built copy with LF line endings; build again when ROS package source changes. This preflight supplements the existing build instructions; it does not establish freshness of an arbitrary install overlay.
- The dashboard ports 18765/18766 and XRCE UDP port 8888 must be free. The worker fails before launching if they are occupied. Each chapter gets its own Gazebo partition, ROS domain and process ownership tag. ROS communication stays on localhost. The worker rejects a selected ROS domain already present in a readable local process environment.

## Record and assemble

- Run from Windows PowerShell in the repository directory. Select a **new** output directory outside the repository and substitute your built Linux root:

  ```powershell
  New-Item -ItemType Directory -Force '..\demo_recordings' | Out-Null
  node scripts/record_demo_video.cjs `
    --linux-root /home/rylanwwade/uav-demo-review-5990c7b `
    --output "$((Resolve-Path '..\demo_recordings').Path)\take-001"
  ```

- The default silent MP4 lasts approximately four minutes: 165 seconds of navigation/localization followed immediately by 75 seconds of ML. The ML header explicitly identifies the fresh simulation. The layout keeps the live dashboard beside the synchronized Gazebo view. The ML GUI uses a closer camera covering the straight drive; this changes only the presentation camera.
- Controls remain tied to actual simulation states and acquisition timestamps. Each chapter is continuous and chronological. Assembly applies one disclosed constant wall-time playback factor per chapter; it never reconstructs model values, changes scores, freezes dashboard states or inserts fake measurement frames. Software rendering can make wall time substantially longer than simulation time. The displayed playback factor is relative to recorded wall time, not `/clock`.
- `manifest.json` contains source paths, capture start times, offsets, durations, playback factors, cue times, both rehearsal summaries and final ffprobe output. `source_timeline.jsonl` maps sampled simulation time to UTC capture time. Cross-source alignment uses process-start timestamps with approximately one source second of startup uncertainty; it is presentation alignment, not a measurement synchronization claim.
- `voiceover-cues.md` provides narration cues with full `HH:MM:SS.mmm` video start/end times, chapter offsets, matching simulation event times and playback speed. Reassembly regenerates this sidecar from the actual captured events.
- Initial idle startup is trimmed from the assembled video. The editor requests four wall seconds before the first observed `NAVIGATE` and three wall seconds before the ML clean-score highlight and fault injection. It clamps to the captured range and snaps forward to an actual Gazebo frame timestamp when capture startup contains a gap. `raw_capture` and `presentation_window` in the manifest retain the original start, requested/actual in-point, source offsets and reason. All later events remain continuous. Raw startup remains in the original captures.
- Output frame quantization can shorten a chapter by a few frames. The manifest records each encoded duration; the cue sidecar uses those measured durations for chapter offsets. It does not assume an exact 165-second boundary or extend footage with frozen final frames.
- Override `--navigation-seconds 165 --ml-seconds 75` during recording to allocate different narration time. A real final shot stays live for five wall seconds after ML arrival. Its displayed duration follows the chapter playback factor.
- To change framing/cues or re-encode existing successful captures without running either simulation again:

  ```powershell
  node scripts/record_demo_video.cjs `
    --output "$((Resolve-Path '..\demo_recordings\take-001').Path)" `
    --assemble-only true
  ```

## Evidence and shutdown

- `uav-ugv-demo.mp4`: 1920x1080, H.264, one video stream, no audio. Add narration separately.
- `navigation/` and `ml/`: original `gazebo.mkv`, headless browser WebM, live start/end dashboard screenshots, final browser DOM state, worker/capture/simulation logs, source timeline and `evidence/{summary.json,samples.jsonl}`. The existing navigation and ML checks must both pass before final assembly.
- `worker.json`: actual display, ROS domain, partition/session tag, source hashes, capture timing, signalled PIDs and `cleanup_remaining_pids`. A successful run requires no remaining tagged processes. Inspect final screenshots and several video frames to check readability, physical robot motion, learned hold, clear flag, resumed driving and arrival.
- Ctrl+C requests worker shutdown through an output-directory stop file. The worker closes ffmpeg normally, then terminates and, if needed, kills only processes with its exact private ownership tag. This includes detached PX4 descendants, the GUI/server, Openbox and Xvfb. The controller closes its own headless browser. Failed runs retain logs, raw footage and a failure manifest; use a new output directory for another recording.
- During assembly, Ctrl+C terminates only the controller's owned Windows ffmpeg encoders and retains the original captures for reassembly.
- If Windows forcibly terminates the controller rather than delivering Ctrl+C, create an empty `stop` file in the active chapter directory. The Linux worker also has bounded startup, browser-handshake and rehearsal deadlines. Do not stop unrelated simulators or shut down the WSL distribution.

## Script checks

```powershell
node --check scripts/record_demo_video.cjs
wsl -d Ubuntu-22.04 -e python3 -m py_compile "/mnt/c/Users/Rylan Wade/Desktop/CS513-CPS/uav_ugv_project/scripts/record_demo_virtual.py"
wsl -d Ubuntu-22.04 -e python3 "/mnt/c/Users/Rylan Wade/Desktop/CS513-CPS/uav_ugv_project/scripts/test_record_demo_virtual.py"
```

- Recording orchestration changes no ROS packages, estimator behavior, model weights or detector thresholds. ROS package builds and registered `colcon test` checks remain required for changes to those packages.

## Recorded validation

- The 2026-11-24 Windows/Ubuntu-22.04 test uses private display `:100`, Mesa D3D12 on Intel UHD Graphics, headless Chromium and the existing seed 513 conditions. Hardware GL works on the private Xvfb display; the worker leaves Mesa driver selection automatic and limits llvmpipe fallback to two threads. Forced llvmpipe stalls the simulation clock and produces stale-pose holds; the validated capture uses hardware acceleration. Software-only recording remains unvalidated.
- Navigation passes all existing rehearsal checks: first-goal truth distance 0.296 m, live and timed hazards observed, restored controls, removed live block, return goal observed, and uncertainty growth/recovery. The driver records 144.26 wall seconds and ends at 134.62 simulation seconds.
- ML passes clean score below 0.50, three flagged decision windows, learned `HOLD_FAULT`, valid pose guard, navigation recovery and arrival. Its final live screenshot shows **Goal reached**, score 0, persistence 0/3 and **Clear**; the final timeline has no fault flags. UAV-timeout injection is excluded. Both workers report empty remaining-process lists.
- The final MP4 contains exactly 240.000 seconds of 1920x1080 H.264 at 24 fps, with one video stream and no audio. Full decode reports no errors. Script syntax checks and all three process-isolation/occupied-port tests pass.
- Local artifacts are retained outside Git at `../demo_recordings/virtual-20261124-take4/`, including the silent MP4, voiceover cues, raw captures, summaries and cleanup evidence. Live end screenshots, the final timeline and rehearsal highlights provide this take's browser-state evidence. `cleanup-audit.json` confirms free capture/dashboard/XRCE resources and records current script hashes.

## Annotate the existing edit

- `scripts/annotate_demo_video.cjs` adds timed vector explanations to the existing MP4. It uses Windows Node, Playwright Chromium, ffmpeg and ffprobe from the dependencies above; it starts no simulation or recording. The [annotation timeline](../scripts/config/demo_annotations.json) identifies the unannotated 240-second `virtual-20261124-take4` source by SHA-256 and uses its 1920x1080 layout.
- Run from the repository in PowerShell, with a new output filename outside Git. Preview mode composites each explanation over its matching original frame without encoding an MP4:

  ```powershell
  node --check scripts/annotate_demo_video.cjs
  node scripts/annotate_demo_video.cjs `
    --input '..\demo_recordings\virtual-20261124-take4\uav-ugv-demo.mp4' `
    --output '..\demo_recordings\virtual-20261124-take4\uav-ugv-demo-explained.mp4' `
    --mode preview
  ```

- Inspect the five `preview-*.png` images listed in the adjacent `.preview.json`. Encode the reviewed timeline with the same unused output name:

  ```powershell
  node scripts/annotate_demo_video.cjs `
    --input '..\demo_recordings\virtual-20261124-take4\uav-ugv-demo.mp4' `
    --output '..\demo_recordings\virtual-20261124-take4\uav-ugv-demo-explained.mp4'
  ```

- The renderer rejects identical input/output paths and existing output/sidecar files. The default timeline accepts only the unannotated original with SHA-256 `a31f8aca96d12792ebb39f74bfdb8ea29e7a32232e2a14b717b7dfbb479ab85c`. It verifies the original and both existing annotated MP4 hashes before and after preview/render. The new MP4 preserves chronological playback, source frame count, existing captions and wall-time disclosures. It adds no audio and changes no measurements, model outputs or navigation states. Encoding uses H.264 CRF 18; pixels outside the overlays can differ through video compression.
- Five nonoverlapping explanations establish one robot with two estimates and the controlling estimate (3–13 s), the downward-camera map and planned route versus position histories (15–25 s), noisy UGV GPS dots (40–48 s), measured wheel speed paired with controller commands (166–176 s), and the 100-sample checks counted by 3/3 (176–186 s). The early track key appears before the first detour. The yellow key identifies fading GPS fixes without pointing at a disappearing dot. ML explanations occupy empty Gazebo ground below the robot and keep all plots, scores, detector flags and supervisor states visible.
- Each wheel-model window pairs the latest 100 measured forward-speed samples with drive commands interpolated to their acquisition times. Inference runs every 0.5 simulation seconds over overlapping windows. Persistence counts three distinct consecutive flagged decisions with increasing decision stamps; repeated windows do not count again. The score is a classifier output, not calibrated confidence. These are the existing [detector](../ros2_ws/src/fault_management/scripts/sensor_fault_detector.py) and [reference/persistence](../ros2_ws/src/fault_management/scripts/sensor_fault_model.py) semantics; the overlays change presentation only.
- The adjacent `uav-ugv-demo-explained.annotations.json` records all text, target descriptions, purposes, pixel coordinates and time spans; configuration/source/result SHA-256 hashes; protected-video hash equality; ffprobe metadata; exact duration, geometry, frame count and silence checks; and full-decode status. `uav-ugv-demo-explained.annotations.md` supplies the human-readable explanation timeline for later voiceover. The original `voiceover-cues.md` retains the underlying event cues.
- A uniquely named adjacent artifacts directory contains SVG/transparent PNG overlays, exact FFmpeg arguments, render/decode logs, five `review-*.png` images from the encoded MP4, and eight `outside-*.png` images without callouts. Browser bounds checks reject text that exceeds its panel. The renderer supports one to eight nonoverlapping callouts, six to ten seconds each, with up to six text lines and configurable color runs. The default edit uses five.
- Annotation changes touch no ROS package. Validation consists of script syntax, previews, the actual full encode, ffprobe checks for 240 seconds / 1920x1080 / 24 fps / 5760 frames / no audio, full FFmpeg decode, timeline/text-boundary checks, all three original-video hash comparisons, overwrite/source rejection checks, and visual review.
