#!/usr/bin/env node
'use strict';
// Windows coordinator. Simulation and Gazebo capture run only in Linux; the
// headless browser captures its own dashboard surface, never a desktop window.
const fs = require('node:fs');
const path = require('node:path');
const {spawn, spawnSync} = require('node:child_process');
const {chromium} = require('playwright');
const ownedEncoders = new Set();

const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const json = file => JSON.parse(fs.readFileSync(file, 'utf8'));
const save = (file, data) => fs.writeFileSync(file, JSON.stringify(data, null, 2) + '\n');
const linuxPath = value => {
  const resolved = path.resolve(value).replaceAll('\\', '/');
  if (!/^[A-Za-z]:\//.test(resolved)) throw Error('Output/scripts must be on a Windows drive');
  return '/mnt/' + resolved[0].toLowerCase() + resolved.slice(2);
};

function options(argv) {
  const result = {distro: 'Ubuntu-22.04', 'navigation-seconds': 165, 'ml-seconds': 75};
  for (let i = 0; i < argv.length; i += 2) {
    if (!argv[i].startsWith('--') || !argv[i + 1]) throw Error('Expected --name value pairs');
    const name = argv[i].slice(2);
    if (!['output', 'linux-root', 'distro', 'navigation-seconds', 'ml-seconds', 'assemble-only'].includes(name))
      throw Error('Unknown option: ' + name);
    result[name] = argv[i + 1];
  }
  if (!result.output || (!result['linux-root'] && result['assemble-only'] !== 'true')) throw Error(
    'Usage: node scripts/record_demo_video.cjs --linux-root /home/USER/BUILT_COPY --output C:\\path\\fresh-take');
  for (const key of ['navigation-seconds', 'ml-seconds']) {
    result[key] = Number(result[key]);
    if (!(result[key] >= 20 && result[key] <= 600)) throw Error('Invalid duration: ' + key);
  }
  result.output = path.resolve(result.output);
  return result;
}

function executable(command, args, logPath, cwd) {
  const log = fs.openSync(logPath, 'w');
  const child = spawn(command, args, {cwd, windowsHide: true, stdio: ['ignore', log, log]});
  if (command === 'ffmpeg') ownedEncoders.add(child);
  fs.closeSync(log);
  // Attach rejection immediately: startup errors must not become unhandled promises.
  child.result = new Promise(resolve => {
    child.once('error', error => { child.finished = true; ownedEncoders.delete(child); resolve({error}); });
    child.once('exit', (code, signal) => {
      child.finished = true; ownedEncoders.delete(child); resolve({code, signal});
    });
  });
  return child;
}

async function completed(child, label) {
  const result = await child.result;
  if (result.error || result.code !== 0) throw Error(`${label}: ${result.error || JSON.stringify(result)}`);
}

function probe(file) {
  const result = spawnSync('ffprobe', ['-v', 'error', '-show_streams', '-show_format', '-of', 'json', file],
                           {encoding: 'utf8', windowsHide: true});
  if (result.status !== 0) throw Error('ffprobe failed: ' + result.stderr);
  return JSON.parse(result.stdout);
}

let interrupted = false;
const interrupt = () => {
  interrupted = true;
  for (const child of ownedEncoders) child.kill();
};
process.on('SIGINT', interrupt);
process.on('SIGTERM', interrupt);

async function captureChapter(opts, chapter, ordinal) {
  const dir = path.join(opts.output, chapter);
  const port = 18765 + ordinal;
  const domain = 140 + (process.pid % 50) + ordinal;
  const worker = executable('wsl.exe', ['-d', opts.distro, '-e', 'python3',
    linuxPath(path.join(__dirname, 'record_demo_virtual.py')),
    '--root', opts['linux-root'], '--output', linuxPath(dir), '--chapter', chapter,
    '--port', String(port), '--domain', String(domain)],
    path.join(opts.output, `${chapter}-worker.log`), opts.output);
  let browser, context, page, browserStart, video;
  try {
    const deadline = Date.now() + 210000;
    while (!fs.existsSync(path.join(dir, 'ready.json'))) {
      if (interrupted) throw Error('Recording interrupted');
      if (worker.finished) { await completed(worker, chapter); throw Error('No readiness metadata'); }
      if (Date.now() > deadline) throw Error('Private display/dashboard startup deadline');
      await sleep(250);
    }
    const ready = json(path.join(dir, 'ready.json'));
    if (!/^:[1-9]\d*$/.test(ready.display) || ready.wayland_display !== null)
      throw Error('Worker did not confirm an isolated X11 display');
    browser = await chromium.launch({headless: true});
    context = await browser.newContext({viewport: {width: 1216, height: 1026},
      recordVideo: {dir: path.join(dir, 'browser'), size: {width: 1216, height: 1026}},
      reducedMotion: 'reduce'});
    browserStart = Date.now();
    page = await context.newPage();
    video = page.video();
    await page.goto(`http://127.0.0.1:${port}`, {waitUntil: 'domcontentloaded', timeout: 30000});
    await page.waitForFunction(() => typeof s !== 'undefined' && s !== null && selectedView !== null,
                              null, {timeout: 30000});
    await page.screenshot({path: path.join(dir, 'dashboard-start.png')});
    save(path.join(dir, 'browser-ready.json'), {capture_start_utc_ms: browserStart,
      ready_utc_ms: Date.now(), viewport: {width: 1216, height: 1026}, headless: true});
    console.log(`${chapter}: private ${ready.display}, ROS domain ${domain}, dashboard ${port}`);
    const trialDeadline = Date.now() + 940000;
    while (!worker.finished) {
      if (interrupted) throw Error('Recording interrupted');
      if (Date.now() > trialDeadline) throw Error('Chapter deadline');
      if (fs.existsSync(path.join(dir, 'rehearsal-finished.json')) &&
          !fs.existsSync(path.join(dir, 'dashboard-end.png'))) {
        await page.screenshot({path: path.join(dir, 'dashboard-end.png')});
        save(path.join(dir, 'dashboard-final-state.json'), await page.evaluate(() => ({
          t: s.t, mode: s.mode, reason: s.reason, flags: s.flags,
          guard_valid: s.guard_valid, ml: s.ml, controls: s.controls, selectedView,
        })));
      }
      await sleep(500);
    }
    await completed(worker, chapter);
  } finally {
    // A file handshake survives Windows/WSL signal forwarding and separate PX4 sessions.
    if (fs.existsSync(dir) && !worker.finished) fs.writeFileSync(path.join(dir, 'stop'), 'stop\n');
    if (context) await context.close();
    if (browser) await browser.close();
    if (!worker.finished) {
      const outcome = await Promise.race([worker.result, sleep(35000).then(() => null)]);
      if (!outcome) console.error(`Cleanup deadline; inspect ${dir}/worker.json before another run.`);
    }
  }
  const browserFile = await video.path();
  const workerInfo = json(path.join(dir, 'worker.json'));
  const evidence = json(path.join(dir, 'evidence', 'summary.json'));
  if (!workerInfo.success || workerInfo.cleanup_remaining_pids.length || !evidence.success)
    throw Error('Chapter evidence or cleanup failed: ' + chapter);
  const gazeboFile = path.join(dir, 'gazebo.mkv');
  const browserProbe = probe(browserFile), gazeboProbe = probe(gazeboFile);
  const start = Math.max(browserStart, workerInfo.gazebo_capture_start_utc_ms);
  const browserTrim = (start - browserStart) / 1000;
  const gazeboTrim = (start - workerInfo.gazebo_capture_start_utc_ms) / 1000;
  const duration = Math.min((workerInfo.capture_end_utc_ms - start) / 1000,
    Number(browserProbe.format.duration) - browserTrim, Number(gazeboProbe.format.duration) - gazeboTrim);
  if (duration < 10) throw Error('Insufficient captured footage');
  return {chapter, dir, browserFile, gazeboFile, browserStart, workerInfo, evidence,
    browserTrim, gazeboTrim, start_utc_ms: start, source_duration_s: duration,
    target_duration_s: opts[chapter + '-seconds'],
    output_seconds_per_source_second: opts[chapter + '-seconds'] / duration,
    browser_probe: browserProbe, gazebo_probe: gazeboProbe};
}

function cuesFor(take) {
  const timeline = fs.readFileSync(path.join(take.dir, 'source_timeline.jsonl'), 'utf8')
    .trim().split('\n').map(JSON.parse);
  const toOutput = t => {
    const near = timeline.reduce((best, row) => Math.abs(row.t - t) < Math.abs(best.t - t) ? row : best);
    return Math.max(0, Math.min(take.target_duration_s,
      (near.utc_ms - take.start_utc_ms) / 1000 * take.output_seconds_per_source_second));
  };
  let cues;
  if (take.chapter === 'navigation') {
    const captions = {
      noise: 'First destination reached\nReturn goal; sensor noise x3\nCompare both error traces.',
      nominal: 'Noise restored to x1',
      gps_off: 'UGV GPS disabled\nUAV retains independent\nlocalization and mapping.',
      corrections_off: 'UAV corrections disabled\nMapping continues.\nWatch pose uncertainty.',
      corrections_on: 'UAV corrections restored\nUncertainty recovers.\nNavigation can resume.',
      sensors_restored: 'Sensors nominal\nLive block removed.\nNavigation chapter complete.',
    };
    cues = [{time_s: 0, text: 'Camera-assisted navigation\nCamera map + A* route\nLive Gazebo view above.'},
      ...take.evidence.actions.map(action => ({time_s: toOutput(action.t),
        simulation_s: action.t,
        text: action.path === '/block' ? 'Live obstacle placed\nRed outline = placement\nGray cells = camera detection'
          : action.path === '/goal' ? 'First destination reached\nReturn goal selected'
          : captions[action.phase] || action.phase}))];
  } else {
    const names = {clean: 'Clean wheel input\nMeasured speed follows\nthe command reference.',
      persistence_1: 'Wheel error +60%\nFirst flagged model decision',
      persistence_2: 'Wheel error +60%\nSecond flagged decision',
      persistence_3: 'Three flagged decisions\nPublished wheel fault',
      detector_hold: 'Learned wheel fault\nSupervisor holds the robot\nPose guard remains valid.',
      navigation_resumes: 'Wheel fault cleared\nSupervisor resumes driving',
      goal: 'Goal reached\nWheel fault clear\nCompleted drive'};
    cues = [{time_s: 0, text: 'Live ML fault detection\nFresh straight-drive simulation\nFixed model and threshold.'},
      ...Object.entries(take.evidence.highlights).filter(([key]) => names[key])
        .map(([key, value]) => ({time_s: toOutput(value.t), simulation_s: value.t, text: names[key]})),
      ...take.evidence.actions.filter(action => action.value === 1.0).map(action => ({
        time_s: toOutput(action.t), simulation_s: action.t,
        text: 'Wheel error removed\nWatch clean model decisions\nand supervisor recovery.'}))];
  }
  cues.sort((a, b) => a.time_s - b.time_s);
  return cues.map((cue, i) => ({...cue, end_s: cues[i + 1]?.time_s ?? take.target_duration_s}));
}

function selectPresentationWindow(take) {
  const timeline = fs.readFileSync(path.join(take.dir, 'source_timeline.jsonl'), 'utf8')
    .trim().split('\n').map(JSON.parse);
  take.raw_capture ??= {start_utc_ms: take.start_utc_ms, source_duration_s: take.source_duration_s,
    browser_trim_s: take.browserTrim, gazebo_trim_s: take.gazeboTrim};
  const raw = take.raw_capture;
  const context = take.chapter === 'navigation' ? 4 : 3;
  const anchor = take.chapter === 'navigation' ? timeline.find(row => row.mode === 'NAVIGATE') :
    timeline.reduce((best, row) => Math.abs(row.t - take.evidence.highlights.clean.t) <
      Math.abs(best.t - take.evidence.highlights.clean.t) ? row : best);
  if (!anchor) throw Error('Missing presentation start anchor: ' + take.chapter);
  const requestedTrim = Math.max(0, Math.min(raw.source_duration_s - 10,
    (anchor.utc_ms - raw.start_utc_ms) / 1000 - context));
  // x11grab may have timestamp gaps during GUI startup. Select a real source
  // frame rather than letting STARTPTS silently move the Gazebo clip forward.
  const packets = spawnSync('ffprobe', ['-v', 'error', '-select_streams', 'v:0',
    '-show_packets', '-show_entries', 'packet=pts_time', '-of', 'json', take.gazeboFile],
    {encoding: 'utf8', windowsHide: true, maxBuffer: 16 * 1024 * 1024});
  if (packets.status !== 0) throw Error('Cannot inspect capture frame timestamps: ' + packets.stderr);
  const firstFrame = JSON.parse(packets.stdout).packets.map(packet => Number(packet.pts_time))
    .find(pts => pts >= raw.gazebo_trim_s + requestedTrim - 0.000001);
  if (!Number.isFinite(firstFrame)) throw Error('No captured frame at presentation in-point');
  const trim = firstFrame - raw.gazebo_trim_s;
  take.start_utc_ms = raw.start_utc_ms + trim * 1000;
  take.browserTrim = raw.browser_trim_s + trim;
  take.gazeboTrim = raw.gazebo_trim_s + trim;
  take.source_duration_s = raw.source_duration_s - trim;
  take.output_seconds_per_source_second = take.target_duration_s / take.source_duration_s;
  take.presentation_window = {initial_idle_trim_s: trim, context_before_anchor_wall_s: context,
    actual_context_before_anchor_wall_s: (anchor.utc_ms - take.start_utc_ms) / 1000,
    requested_initial_idle_trim_s: requestedTrim, gazebo_first_frame_pts_s: firstFrame,
    anchor_simulation_s: anchor.t, anchor_utc_ms: anchor.utc_ms,
    reason: take.chapter === 'navigation' ? 'Lead-in before first NAVIGATE' : 'Lead-in before clean score / injection'};
}

async function assemble(opts, take, index) {
  if (interrupted) throw Error('Assembly interrupted');
  selectPresentationWindow(take);
  const duration = take.target_duration_s;
  const factor = take.output_seconds_per_source_second;
  take.cues = cuesFor(take);
  const label = index === 0 ? '1  Camera-assisted navigation and localization'
                            : '2  Live ML fault detection and recovery | fresh simulation';
  const staticText = [label,
    'Gazebo simulation',
    `Chronological playback\n${(1 / factor).toFixed(2)}x recorded wall-time speed`,
    'White: actual robot\nBlue: cooperative estimate\nOrange: baseline estimate\nTruth is display/scoring only.'];
  const name = take.chapter;
  staticText.forEach((text, i) => fs.writeFileSync(path.join(opts.output, `${name}-text-${i}.txt`), text));
  const draw = (file, x, y, size, color = 'white', enabled = '') =>
    `drawtext=fontfile='C\\:/Windows/Fonts/arial.ttf':textfile='${file}':expansion=none:` +
    `x=${x}:y=${y}:fontsize=${size}:fontcolor=${color}:line_spacing=12${enabled}`;
  const filters = [
    `[0:v]trim=start=${take.gazeboTrim}:duration=${take.source_duration_s},setpts=(PTS-${take.gazeboTrim}/TB)*${factor},` +
      'crop=1200:844:0:0,scale=704:496,pad=704:528:0:16:color=0x111821,fps=24[gazebo]',
    `[1:v]trim=start=${take.browserTrim}:duration=${take.source_duration_s},setpts=(PTS-${take.browserTrim}/TB)*${factor},` +
      'fps=24[dashboard]',
    `color=c=0x111821:s=1920x1080:r=24:d=${duration}[base]`,
    '[base][gazebo]overlay=0:54:shortest=1[a]',
    '[a][dashboard]overlay=704:54:shortest=1[b]',
  ];
  let draws = [draw(`${name}-text-0.txt`, 22, 13, 25), draw(`${name}-text-1.txt`, 24, 608, 24),
    draw(`${name}-text-2.txt`, 24, 956, 20, '0x9baec1')];
  if (index === 0) draws.push(draw(`${name}-text-3.txt`, 24, 824, 20, '0x9baec1'));
  for (let i = 0; i < take.cues.length; i++) {
    const cue = take.cues[i];
    const file = `${name}-cue-${i}.txt`;
    fs.writeFileSync(path.join(opts.output, file), cue.text);
    // Half-open intervals prevent the previous cue being drawn on a boundary.
    draws.push(draw(file, 24, 704, 25, '0xcce6f5',
      `:enable='gte(t,${cue.time_s})*lt(t,${cue.end_s})'`));
  }
  filters.push('[b]' + draws.join(',') + '[out]');
  const filterFile = path.join(opts.output, name + '-filter.txt');
  fs.writeFileSync(filterFile, filters.join(';\n'));
  const target = path.join(opts.output, name + '-edited.mp4');
  const child = executable('ffmpeg', ['-hide_banner', '-y', '-i', take.gazeboFile,
    '-i', take.browserFile, '-filter_complex', fs.readFileSync(filterFile, 'utf8'),
    '-map', '[out]', '-an', '-t', String(duration), '-c:v', 'libx264', '-preset', 'fast',
    '-crf', '20', '-threads', '4', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', target],
    path.join(opts.output, name + '-edit.log'), opts.output);
  await completed(child, 'Assembly ' + name);
  take.encoded_duration_s = Number(probe(target).format.duration);
  if (Math.abs(take.encoded_duration_s - duration) > 0.25)
    throw Error(`Encoded ${name} duration differs from pacing by over one quarter second`);
  for (const cue of take.cues) cue.end_s = Math.min(cue.end_s, take.encoded_duration_s);
  return target;
}

function writeVoiceoverCues(opts, manifest) {
  const timestamp = seconds => new Date(Math.round(seconds * 1000)).toISOString().slice(11, 23);
  const lines = ['# Voiceover cues', '',
    '- Times refer to the edited video, including chapter offsets. They are playback times, not simulation time.',
    '- Each chapter preserves chronological source footage and uses the constant wall-time speed shown below.',
    '- Cue text describes what to explain; model scores and flags remain the live dashboard outputs.', ''];
  let offset = 0;
  for (const take of manifest.chapters) {
    take.output_start_s = offset;
    lines.push(`## ${take.chapter === 'navigation' ? 'Navigation and localization' : 'ML fault detection and recovery'}`,
      '', `- Chapter start: ${timestamp(offset)}. Encoded duration: ${take.encoded_duration_s.toFixed(3)} seconds.`,
      `- Playback: ${(1 / take.output_seconds_per_source_second).toFixed(4)}x recorded wall-time speed.`,
      '', '| Video start | Video end | Simulation event time | What to explain |',
      '| --- | --- | --- | --- |');
    for (const cue of take.cues) {
      if (cue.end_s - cue.time_s <= 0.001) continue;
      lines.push(`| ${timestamp(offset + cue.time_s)} | ${timestamp(offset + cue.end_s)} | ` +
        `${cue.simulation_s === undefined ? 'Chapter opening' : cue.simulation_s.toFixed(3) + ' s'} | ` +
        cue.text.replaceAll('\n', ' / ').replaceAll('|', '\\|') + ' |');
    }
    lines.push('');
    offset += take.encoded_duration_s;
  }
  const target = path.join(opts.output, 'voiceover-cues.md');
  fs.writeFileSync(target, lines.join('\n'));
  manifest.voiceover_cues = target;
}

async function main() {
  const opts = options(process.argv.slice(2));
  for (const name of ['ffmpeg', 'ffprobe']) {
    const result = spawnSync(name, ['-version'], {windowsHide: true, stdio: 'ignore'});
    if (result.status !== 0) throw Error('Missing Windows dependency: ' + name);
  }
  if (!fs.existsSync(chromium.executablePath())) throw Error('Playwright Chromium is not installed');
  const reuse = opts['assemble-only'] === 'true';
  if (!reuse) fs.mkdirSync(opts.output, {recursive: false});
  const manifest = reuse ? json(path.join(opts.output, 'manifest.json')) :
    {created_utc: new Date().toISOString(), options: opts, success: false,
    capture: 'Private Xvfb X11 display plus headless Chromium dashboard surface. No desktop/audio capture.',
    timing: 'UTC process-start alignment; startup uncertainty approximately one source second. Initial idle startup is trimmed with a recorded lead-in. Each selected chapter uses one continuous wall-time scaling. Cues match sampled simulation timestamps; no score or flag is synthesized.',
    chapters: []};
  manifest.success = false;
  manifest.timing = 'UTC process-start alignment; startup uncertainty approximately one source second. Initial idle startup is trimmed to an actual Gazebo frame PTS. Selected footage is continuous with one constant wall-time scaling per chapter. Encoded chapter durations determine cue offsets. No scores, flags or frozen-frame segments are synthesized.';
  if (reuse) manifest.reassembled_utc = new Date().toISOString();
  try {
    if (!reuse) {
      for (const [i, chapter] of ['navigation', 'ml'].entries()) {
        const take = await captureChapter(opts, chapter, i);
        manifest.chapters.push(take);
        save(path.join(opts.output, 'manifest.json'), manifest);
      }
    } else if (manifest.chapters.length !== 2 || manifest.chapters.some(take => !take.evidence.success)) {
      throw Error('Reassembly requires two successful captured chapters');
    }
    const edited = [];
    for (const [i, take] of manifest.chapters.entries()) edited.push(await assemble(opts, take, i));
    if (interrupted) throw Error('Assembly interrupted');
    writeVoiceoverCues(opts, manifest);
    fs.writeFileSync(path.join(opts.output, 'concat.txt'), edited.map(file =>
      "file '" + path.basename(file) + "'").join('\n') + '\n');
    const target = path.join(opts.output, 'uav-ugv-demo.mp4');
    await completed(executable('ffmpeg', ['-hide_banner', '-y', '-f', 'concat', '-safe', '0',
      '-i', 'concat.txt', '-c', 'copy', '-an', '-movflags', '+faststart', target],
      path.join(opts.output, 'assembly.log'), opts.output), 'Final concatenation');
    manifest.output = {path: target, probe: probe(target)};
    if (interrupted) throw Error('Assembly interrupted');
    const streams = manifest.output.probe.streams;
    if (streams.length !== 1 || streams[0].codec_type !== 'video' ||
        streams[0].width !== 1920 || streams[0].height !== 1080)
      throw Error('Final video must contain one 1920x1080 video stream and no audio');
    manifest.success = true;
    delete manifest.failure;
    console.log('Completed: ' + target);
  } catch (error) {
    manifest.failure = String(error.stack || error);
    throw error;
  } finally {
    save(path.join(opts.output, 'manifest.json'), manifest);
  }
}

if (require.main === module) main().catch(error => {console.error(error); process.exitCode = 1;});
module.exports = {options, linuxPath, cuesFor, selectPresentationWindow};
