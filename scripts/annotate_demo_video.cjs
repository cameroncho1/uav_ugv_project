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
  const result = {config: path.join(__dirname, 'config', 'demo_annotations.json')};
  for (let i = 0; i < argv.length; i += 2) {
    const name = argv[i].slice(2);
    if (!argv[i].startsWith('--') || !['input', 'output', 'config'].includes(name) || !argv[i + 1])
      throw Error('Usage: node scripts/annotate_demo_video.cjs --input ORIGINAL.mp4 --output NEW.mp4 [--config TIMELINE.json]');
    result[name] = path.resolve(argv[i + 1]);
  }
  if (!result.input || !result.output) throw Error('Both --input and --output are required');
  result.input = fs.realpathSync(result.input);
  result.output = path.resolve(result.output);
  if (samePath(result.input, result.output)) throw Error('Input and output must be different files');
  if (path.extname(result.output).toLowerCase() !== '.mp4') throw Error('Output must end in .mp4');
  if (fs.existsSync(result.output)) throw Error('Refusing to overwrite existing output: ' + result.output);
  result.manifest = result.output.replace(/\.mp4$/i, '.annotations.json');
  if (fs.existsSync(result.manifest)) throw Error('Refusing to overwrite existing manifest: ' + result.manifest);
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
  if (config.callouts.length !== 5) throw Error('Exactly five callouts are required');
  const ids = new Set();
  let previousEnd = 0;
  for (const c of config.callouts) {
    if (!c.id || ids.has(c.id)) throw Error('Callout IDs must be unique');
    ids.add(c.id);
    if (![c.start_s, c.end_s, c.review_s].every(Number.isFinite) || c.start_s < previousEnd ||
        c.end_s - c.start_s < 4 || c.end_s - c.start_s > 6 || c.end_s > config.expected_video.duration_s ||
        c.review_s <= c.start_s || c.review_s >= c.end_s) throw Error('Invalid or overlapping callout timing: ' + c.id);
    previousEnd = c.end_s;
    if (!Array.isArray(c.text) || !c.text.length || c.text.length > 2) throw Error('Expected one or two text lines');
    const b = c.box;
    if (![b.x, b.y, b.width, b.height].every(Number.isFinite) || b.x < 0 || b.y < 0 ||
        b.width <= 0 || b.height <= 0 || b.x + b.width > config.expected_video.width ||
        b.y + b.height > config.expected_video.height) throw Error('Invalid callout box: ' + c.id);
    if (!Array.isArray(c.pointer) || c.pointer.length < 2 || c.pointer.some(p => p.length !== 2 ||
        !p.every(Number.isFinite) || p[0] < 0 || p[0] > config.expected_video.width ||
        p[1] < 0 || p[1] > config.expected_video.height)) throw Error('Invalid pointer: ' + c.id);
  }
}

function svg(config, c) {
  const b = c.box, s = config.style;
  const lineHeight = 33;
  const firstY = c.text.length === 1 ? b.y + (b.height - s.font_size) / 2 : b.y + 13;
  const lines = c.text.map((line, i) => {
    const size = c.id === 'two-estimates' && i === 1 ? s.detail_size : s.font_size;
    // The track key uses the same measured-track colors as the captured dashboard.
    const text = c.id === 'two-estimates' && i === 1
      ? `<tspan fill="#55a7fa">Blue: with UAV</tspan><tspan fill="${s.text}"> · </tspan><tspan fill="#ffac50">Orange: without UAV</tspan>`
      : escapeXml(line);
    return `<text x="${b.x + 20}" y="${firstY + i * lineHeight}" font-family="${escapeXml(s.font)}" font-size="${size}" font-weight="500" dominant-baseline="text-before-edge" fill="${s.text}">${text}</text>`;
  }).join('');
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${config.expected_video.width}" height="${config.expected_video.height}" viewBox="0 0 ${config.expected_video.width} ${config.expected_video.height}">
  <polyline points="${c.pointer.map(p => p.join(',')).join(' ')}" fill="none" stroke="${s.border}" stroke-width="${s.pointer_width}" stroke-linecap="round" stroke-linejoin="round"/>
  <rect x="${b.x}" y="${b.y}" width="${b.width}" height="${b.height}" rx="14" fill="${s.panel}" fill-opacity="0.97" stroke="${s.border}" stroke-width="1.5"/>
  ${lines}</svg>`;
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

async function main() {
  const opts = options(process.argv.slice(2));
  const config = readJson(opts.config);
  validateTimeline(config);
  const sourceHash = await sha256(opts.input);
  const inputProbe = probe(opts.input);
  checkVideo(inputProbe, config.expected_video);
  fs.mkdirSync(path.dirname(opts.output), {recursive: true});
  const artifacts = fs.mkdtempSync(path.join(path.dirname(opts.output), path.basename(opts.output, '.mp4') + '-artifacts-'));
  const browser = await chromium.launch({headless: true});
  const overlays = [];
  try {
    const page = await browser.newPage({viewport: {width: config.expected_video.width, height: config.expected_video.height}, deviceScaleFactor: 1});
    for (const c of config.callouts) {
      const markup = svg(config, c);
      fs.writeFileSync(path.join(artifacts, c.id + '.svg'), markup, {flag: 'wx'});
      await page.setContent(`<html><body style="margin:0;background:transparent">${markup}</body></html>`);
      await page.evaluate(() => document.fonts.ready);
      const overflow = await page.locator('text').evaluateAll(nodes => nodes.some(n => n.getBBox().x + n.getBBox().width > Number(n.ownerSVGElement.querySelector('rect').getAttribute('x')) + Number(n.ownerSVGElement.querySelector('rect').getAttribute('width')) - 12));
      if (overflow) throw Error('Callout text exceeds its panel: ' + c.id);
      const png = path.join(artifacts, c.id + '.png');
      await page.screenshot({path: png, omitBackground: true});
      overlays.push(png);
    }
  } finally {await browser.close();}

  const filters = config.callouts.map((c, i) => `[${i === 0 ? '0:v' : 'a' + i}][${i + 1}:v]overlay=0:0:eof_action=repeat:enable='gte(t,${c.start_s})*lt(t,${c.end_s})'[a${i + 1}]`).join(';');
  const args = ['-hide_banner', '-nostdin', '-n', '-i', opts.input, ...overlays.flatMap(png => ['-i', png]),
    '-filter_complex', filters, '-map', '[a5]', '-map_metadata', '0', '-c:v', 'libx264', '-preset', 'medium',
    '-crf', '18', '-pix_fmt', 'yuv420p', '-fps_mode', 'passthrough', '-an', '-movflags', '+faststart', opts.output];
  writeJson(path.join(artifacts, 'render-command.json'), {command: 'ffmpeg', args});
  console.log('Rendering ' + opts.output);
  await run('ffmpeg', args, path.join(artifacts, 'render.log'));
  const outputProbe = probe(opts.output);
  const verifiedVideo = checkVideo(outputProbe, config.expected_video);
  if (verifiedVideo.codec !== 'h264') throw Error('Expected H.264 output');
  await run('ffmpeg', ['-hide_banner', '-nostdin', '-v', 'error', '-xerror', '-i', opts.output, '-map', '0:v:0', '-f', 'null', '-'], path.join(artifacts, 'decode.log'));
  const reviewFrames = [];
  for (const c of config.callouts) {
    const png = path.join(artifacts, 'review-' + c.id + '.png');
    await run('ffmpeg', ['-hide_banner', '-nostdin', '-v', 'error', '-ss', String(c.review_s), '-i', opts.output, '-frames:v', '1', '-n', png], path.join(artifacts, 'review-' + c.id + '.log'));
    reviewFrames.push({id: c.id, playback_s: c.review_s, path: png});
  }
  const inputHashAfter = await sha256(opts.input);
  const originalUnchanged = sourceHash === inputHashAfter;
  const result = {schema_version: 1, created_utc: new Date().toISOString(), input: {path: opts.input, sha256_before: sourceHash, sha256_after: inputHashAfter, probe: inputProbe},
    output: {path: opts.output, sha256: await sha256(opts.output), probe: outputProbe}, original_unchanged: originalUnchanged,
    verification: {...verifiedVideo, full_decode_passed: true, callout_count: config.callouts.length},
    annotation_config: {path: opts.config, sha256: await sha256(opts.config)}, callouts: config.callouts, review_frames: reviewFrames, artifacts};
  writeJson(opts.manifest, result);
  if (!originalUnchanged) throw Error('Input hash changed; see manifest');
  console.log(JSON.stringify({output: opts.output, manifest: opts.manifest, artifacts, original_unchanged: true, verification: result.verification}, null, 2));
}

main().catch(error => {console.error(error.stack || error); process.exitCode = 1;});
