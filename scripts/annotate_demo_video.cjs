#!/usr/bin/env node
'use strict';
// Offline presentation overlays only. Does not launch ROS, Gazebo, or capture.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const {spawn, spawnSync} = require('node:child_process');
const {chromium} = require('playwright');

const readJson = file => JSON.parse(fs.readFileSync(file, 'utf8'));
const writeJson = (file, value) => fs.writeFileSync(file, JSON.stringify(value, null, 2) + '\n', {flag: 'wx'});
const samePath = (a, b) => process.platform === 'win32' ? a.toLowerCase() === b.toLowerCase() : a === b;
const escapeXml = value => String(value).replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;');

function options(argv) {
  const result = {config: path.join(__dirname, 'config', 'demo_annotations.json'), mode: 'render'};
  for (let i = 0; i < argv.length; i += 2) {
    const name = argv[i].slice(2);
    if (!argv[i].startsWith('--') || !['input', 'output', 'config', 'mode'].includes(name) || !argv[i + 1])
      throw Error('Usage: node scripts/annotate_demo_video.cjs --input ORIGINAL.mp4 --output NEW.mp4 [--config TIMELINE.json] [--mode preview|render]');
    result[name] = name === 'mode' ? argv[i + 1] : path.resolve(argv[i + 1]);
  }
  if (!result.input || !result.output) throw Error('Both --input and --output are required');
  if (!['preview', 'render'].includes(result.mode)) throw Error('Mode must be preview or render');
  result.input = fs.realpathSync(result.input);
  result.output = path.resolve(result.output);
  if (samePath(result.input, result.output)) throw Error('Input and output must be different files');
  if (path.extname(result.output).toLowerCase() !== '.mp4') throw Error('Output must end in .mp4');
  if (fs.existsSync(result.output)) throw Error('Refusing to overwrite existing output: ' + result.output);
  result.manifest = result.output.replace(/\.mp4$/i, '.annotations.json');
  result.timeline = result.output.replace(/\.mp4$/i, '.annotations.md');
  result.previewManifest = result.output.replace(/\.mp4$/i, '.preview.json');
  for (const file of [result.manifest, result.timeline, ...(result.mode === 'preview' ? [result.previewManifest] : [])])
    if (fs.existsSync(file)) throw Error('Refusing to overwrite existing sidecar: ' + file);
  return result;
}

async function sha256(file) {
  const hash = crypto.createHash('sha256');
  for await (const chunk of fs.createReadStream(file)) hash.update(chunk);
  return hash.digest('hex');
}

function probe(file) {
  const r = spawnSync('ffprobe', ['-v', 'error', '-show_streams', '-show_format', '-of', 'json', file], {encoding: 'utf8', windowsHide: true});
  if (r.status !== 0) throw Error('ffprobe failed: ' + r.stderr);
  return JSON.parse(r.stdout);
}

function checkVideo(info, expected) {
  const videos = info.streams.filter(s => s.codec_type === 'video');
  if (info.streams.length !== 1 || videos.length !== 1) throw Error('Expected one video stream and no audio/other streams');
  const v = videos[0];
  if (v.width !== expected.width || v.height !== expected.height || v.avg_frame_rate !== expected.frame_rate ||
      Math.abs(Number(info.format.duration) - expected.duration_s) > 0.001 || Number(v.nb_frames) !== expected.frames)
    throw Error('Video geometry, timing, or frame count does not match the annotation timeline');
  return {duration_s: Number(info.format.duration), width: v.width, height: v.height, frame_rate: v.avg_frame_rate,
    frames: Number(v.nb_frames), codec: v.codec_name, audio_streams: 0};
}

function validateTimeline(config) {
  if (!Array.isArray(config.callouts) || config.callouts.length < 1 || config.callouts.length > 8)
    throw Error('Expected one to eight callouts');
  const ids = new Set();
  let previousEnd = 0;
  for (const c of config.callouts) {
    if (!/^[a-z0-9-]+$/.test(c.id) || ids.has(c.id)) throw Error('Callout IDs must be unique filename-safe names');
    ids.add(c.id);
    if (![c.start_s, c.end_s, c.review_s].every(Number.isFinite) || c.start_s < previousEnd ||
        c.end_s - c.start_s < 6 || c.end_s - c.start_s > 10 || c.end_s > config.expected_video.duration_s ||
        c.review_s <= c.start_s || c.review_s >= c.end_s) throw Error('Invalid or overlapping callout timing: ' + c.id);
    previousEnd = c.end_s;
    if (!Array.isArray(c.text) || !c.text.length || c.text.length > 6) throw Error('Expected one to six text lines');
    for (const line of c.text) {
      if (typeof line === 'string') {
        if (!line.trim()) throw Error('Empty text line: ' + c.id);
      } else if (!line || !Array.isArray(line.runs) || !line.runs.length || line.runs.some(run =>
        typeof run.text !== 'string' || !run.text || (run.color && !/^#[0-9a-f]{6}$/i.test(run.color)))) {
        throw Error('Expected text or colored text runs: ' + c.id);
      }
    }
    const b = c.box;
    if (![b.x, b.y, b.width, b.height].every(Number.isFinite) || b.x < 0 || b.y < 0 ||
        b.width <= 0 || b.height <= 0 || b.x + b.width > config.expected_video.width ||
        b.y + b.height > config.expected_video.height) throw Error('Invalid callout box: ' + c.id);
    if (c.pointer && (!Array.isArray(c.pointer) || c.pointer.length < 2 || c.pointer.some(p => p.length !== 2 ||
        !p.every(Number.isFinite) || p[0] < 0 || p[0] > config.expected_video.width ||
        p[1] < 0 || p[1] > config.expected_video.height))) throw Error('Invalid pointer: ' + c.id);
  }
  if (config.outside_review_s && (!Array.isArray(config.outside_review_s) || config.outside_review_s.some(t =>
    !Number.isFinite(t) || t < 0 || t >= config.expected_video.duration_s ||
    config.callouts.some(c => t >= c.start_s && t < c.end_s)))) throw Error('Outside review times must have no callout');
}

function svg(config, c) {
  const b = c.box, s = config.style;
  const lineHeight = s.line_height, firstY = b.y + s.padding_y;
  const lines = c.text.map((line, i) => {
    const runs = typeof line === 'string' ? [{text: line}] : line.runs;
    const text = runs.map(run => `<tspan fill="${run.color || s.text}">${escapeXml(run.text)}</tspan>`).join('');
    return `<text x="${b.x + s.padding_x}" y="${firstY + i * lineHeight}" font-family="${escapeXml(s.font)}" font-size="${i === 0 ? s.heading_size : s.font_size}" font-weight="${i === 0 ? 600 : 500}" dominant-baseline="text-before-edge" fill="${s.text}">${text}</text>`;
  }).join('');
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${config.expected_video.width}" height="${config.expected_video.height}" viewBox="0 0 ${config.expected_video.width} ${config.expected_video.height}">
  ${c.pointer ? `<polyline points="${c.pointer.map(p => p.join(',')).join(' ')}" fill="none" stroke="${s.border}" stroke-width="${s.pointer_width}" stroke-linecap="round" stroke-linejoin="round"/>` : ''}
  <rect x="${b.x}" y="${b.y}" width="${b.width}" height="${b.height}" rx="14" fill="${s.panel}" fill-opacity="0.97" stroke="${s.border}" stroke-width="1.5"/>
  ${lines}</svg>`;
}

const plainText = line => typeof line === 'string' ? line : line.runs.map(run => run.text).join('');
const timestamp = seconds => new Date(Math.round(seconds * 1000)).toISOString().slice(11, 23);

async function preservedVideos(config, input) {
  return Promise.all((config.preserve_existing_videos || []).map(async entry => {
    if (path.basename(entry.filename) !== entry.filename || !/^[a-f0-9]{64}$/.test(entry.sha256))
      throw Error('Invalid preserved video entry');
    const file = path.join(path.dirname(input), entry.filename);
    const before = await sha256(file);
    if (before !== entry.sha256) throw Error('Preserved video does not match its expected hash: ' + file);
    return {path: file, sha256_before: before};
  }));
}

async function verifyPreservation(input, sourceHash, protectedVideos) {
  const after = await sha256(input);
  for (const video of protectedVideos) {
    video.sha256_after = await sha256(video.path);
    video.unchanged = video.sha256_before === video.sha256_after;
  }
  if (sourceHash !== after || protectedVideos.some(video => !video.unchanged)) throw Error('A protected video hash changed');
  return after;
}

function writeTimeline(file, opts, config, sourceHash) {
  const rows = config.callouts.map(c => `| ${timestamp(c.start_s)} | ${timestamp(c.end_s)} | ${c.text.map(plainText).join('<br>')} | ${c.purpose} |`);
  fs.writeFileSync(file, ['# Explanatory annotation and voiceover cues', '',
    `- Video: ${path.basename(opts.output)}. Audio remains absent.`,
    '- Times are playback times. The underlying chapters and wall-time playback factors remain unchanged.',
    '- Read these explanations alongside the live footage. Existing phase captions identify actual events.',
    '- The ML score is a classifier output, not calibrated confidence. Inference runs every 0.5 simulation seconds.',
    '- The 100-sample windows overlap. Persistence counts distinct increasing decision stamps, not sensors, fault types or independent trials.',
    `- Original SHA-256: \`${sourceHash}\`.`, '',
    '| Start | End | On-screen explanation | What this teaches |', '| --- | --- | --- | --- |', ...rows, ''].join('\n'), {flag: 'wx'});
}

async function run(command, args, logFile) {
  const log = fs.openSync(logFile, 'wx');
  try {
    await new Promise((resolve, reject) => {
      const child = spawn(command, args, {windowsHide: true, stdio: ['ignore', log, log]});
      const stop = () => child.kill();
      process.once('SIGINT', stop);
      process.once('SIGTERM', stop);
      const cleanup = () => { process.removeListener('SIGINT', stop); process.removeListener('SIGTERM', stop); };
      child.once('error', error => {cleanup(); reject(error);});
      child.once('exit', (code, signal) => {cleanup(); code === 0 ? resolve() : reject(Error(`${command} failed (${code || signal}); see ${logFile}`));});
    });
  } finally {fs.closeSync(log);}
}

async function extractFrame(input, seconds, png, artifacts, logName) {
  await run('ffmpeg', ['-hide_banner', '-nostdin', '-v', 'error', '-ss', String(seconds), '-i', input, '-frames:v', '1', '-n', png], path.join(artifacts, logName + '.log'));
}

async function main() {
  const opts = options(process.argv.slice(2));
  const config = readJson(opts.config);
  validateTimeline(config);
  const sourceHash = await sha256(opts.input);
  if (config.source_sha256 && sourceHash !== config.source_sha256) throw Error('Input is not the unannotated original identified by this timeline');
  const protectedVideos = await preservedVideos(config, opts.input);
  const inputProbe = probe(opts.input);
  checkVideo(inputProbe, config.expected_video);
  fs.mkdirSync(path.dirname(opts.output), {recursive: true});
  const artifacts = fs.mkdtempSync(path.join(path.dirname(opts.output), path.basename(opts.output, '.mp4') + '-' + opts.mode + '-'));
  const browser = await chromium.launch({headless: true});
  const overlays = [], previews = [];
  try {
    const page = await browser.newPage({viewport: {width: config.expected_video.width, height: config.expected_video.height}, deviceScaleFactor: 1});
    for (const c of config.callouts) {
      const markup = svg(config, c);
      fs.writeFileSync(path.join(artifacts, c.id + '.svg'), markup, {flag: 'wx'});
      await page.setContent(`<html><body style="margin:0;background:transparent">${markup}</body></html>`);
      await page.evaluate(() => document.fonts.ready);
      const overflow = await page.locator('text').evaluateAll(nodes => nodes.some(n => {
        const text = n.getBBox(), rect = n.ownerSVGElement.querySelector('rect');
        const x = Number(rect.getAttribute('x')), y = Number(rect.getAttribute('y'));
        return text.x < x + 10 || text.y < y + 8 ||
          text.x + text.width > x + Number(rect.getAttribute('width')) - 10 ||
          text.y + text.height > y + Number(rect.getAttribute('height')) - 8;
      }));
      if (overflow) throw Error('Callout text exceeds its panel: ' + c.id);
      const png = path.join(artifacts, c.id + '.png');
      await page.screenshot({path: png, omitBackground: true});
      overlays.push(png);
      if (opts.mode === 'preview') {
        const original = path.join(artifacts, 'original-' + c.id + '.png');
        await extractFrame(opts.input, c.review_s, original, artifacts, 'original-' + c.id);
        const background = fs.readFileSync(original).toString('base64');
        await page.setContent(`<html><body style="margin:0;background:url(data:image/png;base64,${background}) no-repeat">${markup}</body></html>`);
        await page.evaluate(() => document.fonts.ready);
        const preview = path.join(artifacts, 'preview-' + c.id + '.png');
        await page.screenshot({path: preview});
        previews.push({id: c.id, playback_s: c.review_s, path: preview});
      }
    }
  } finally {await browser.close();}

  if (opts.mode === 'preview') {
    const after = await verifyPreservation(opts.input, sourceHash, protectedVideos);
    writeJson(opts.previewManifest, {schema_version: 2, mode: 'preview', input: {path: opts.input, sha256_before: sourceHash, sha256_after: after},
      original_unchanged: true, preserved_videos: protectedVideos, annotation_config: {path: opts.config, sha256: await sha256(opts.config)},
      callouts: config.callouts, previews, artifacts});
    console.log(JSON.stringify({preview_manifest: opts.previewManifest, previews, artifacts}, null, 2));
    return;
  }

  const filters = config.callouts.map((c, i) => `[${i === 0 ? '0:v' : 'a' + i}][${i + 1}:v]overlay=0:0:eof_action=repeat:enable='gte(t,${c.start_s})*lt(t,${c.end_s})'[a${i + 1}]`).join(';');
  const args = ['-hide_banner', '-nostdin', '-n', '-i', opts.input, ...overlays.flatMap(png => ['-i', png]),
    '-filter_complex', filters, '-map', `[a${config.callouts.length}]`, '-map_metadata', '0', '-c:v', 'libx264', '-preset', 'medium',
    '-crf', '18', '-pix_fmt', 'yuv420p', '-fps_mode', 'passthrough', '-an', '-movflags', '+faststart', opts.output];
  writeJson(path.join(artifacts, 'render-command.json'), {command: 'ffmpeg', args});
  console.log('Rendering ' + opts.output);
  await run('ffmpeg', args, path.join(artifacts, 'render.log'));
  const outputProbe = probe(opts.output);
  const verifiedVideo = checkVideo(outputProbe, config.expected_video);
  if (verifiedVideo.codec !== 'h264') throw Error('Expected H.264 output');
  await run('ffmpeg', ['-hide_banner', '-nostdin', '-v', 'error', '-xerror', '-i', opts.output, '-map', '0:v:0', '-f', 'null', '-'], path.join(artifacts, 'decode.log'));
  const reviewFrames = [], outsideFrames = [];
  for (const c of config.callouts) {
    const png = path.join(artifacts, 'review-' + c.id + '.png');
    await extractFrame(opts.output, c.review_s, png, artifacts, 'review-' + c.id);
    reviewFrames.push({id: c.id, playback_s: c.review_s, path: png});
  }
  for (const seconds of config.outside_review_s || []) {
    const name = 'outside-' + String(seconds).replace('.', '-'), png = path.join(artifacts, name + '.png');
    await extractFrame(opts.output, seconds, png, artifacts, name);
    outsideFrames.push({playback_s: seconds, path: png});
  }
  const inputHashAfter = await verifyPreservation(opts.input, sourceHash, protectedVideos);
  writeTimeline(opts.timeline, opts, config, sourceHash);
  const result = {schema_version: 2, created_utc: new Date().toISOString(), input: {path: opts.input, sha256_before: sourceHash, sha256_after: inputHashAfter, probe: inputProbe},
    output: {path: opts.output, sha256: await sha256(opts.output), probe: outputProbe}, original_unchanged: true, preserved_videos: protectedVideos,
    verification: {...verifiedVideo, full_decode_passed: true, callout_count: config.callouts.length},
    annotation_config: {path: opts.config, sha256: await sha256(opts.config)}, callouts: config.callouts, review_frames: reviewFrames,
    outside_frames: outsideFrames, voiceover_timeline: opts.timeline, artifacts};
  writeJson(opts.manifest, result);
  console.log(JSON.stringify({output: opts.output, manifest: opts.manifest, timeline: opts.timeline, artifacts, original_unchanged: true, verification: result.verification}, null, 2));
}

main().catch(error => {console.error(error.stack || error); process.exitCode = 1;});
