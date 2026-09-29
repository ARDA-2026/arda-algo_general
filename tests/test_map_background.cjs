// Offline regression: fallback uses geographic polygons, including island holes.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../static/index.html'), 'utf8');
for (const match of html.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g)) {
  new vm.Script(match[1]);
}
const calls = [];
const drawing = new Proxy({}, {get: (_, name) => (...args) => calls.push([name, ...args])});
const river = {features: [{geometry: {type: 'Polygon', coordinates: [
  [[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]],
  [[4, 4], [6, 4], [6, 6], [4, 6], [4, 4]],
]}}]};
const urls = [], images = [];
const sandbox = {ctx: drawing, toCanvas: (lon, lat) => [lon * 10, -lat * 10],
  api: async url => { urls.push(url); return river; },
  Image: class {constructor() { images.push(this); }},
  requestRender() {}, setMapBadge() {}, setText() {}, setTimeout() {}};
vm.createContext(sandbox);
vm.runInContext(html.slice(html.indexOf('    let mapBgLoaded'),
  html.indexOf('    // \uacbd\ub3c4 1')), sandbox);
vm.runInContext(html.slice(html.indexOf('    function drawMapBackground('),
  html.indexOf('    function drawMapScrim(')), sandbox);

(async () => {
  sandbox.setMapBackground('first-area');
  assert.match(images[0].src, /^\/map\/background\?v=/);
  images[0].onerror();
  await sandbox.loadRiverBackground();
  assert.deepEqual(urls, ['/river-geojson']);
  assert.ok(!images[0].src.includes('mapomap.jpg'));
  const map = {lon_min: 0, lat_min: 0, lon_max: 10, lat_max: 10};
  sandbox.drawMapBackground(map);
  assert.equal(calls.filter(c => c[0] === 'drawImage').length, 0);
  assert.equal(calls.filter(c => c[0] === 'closePath').length, 2);
  assert.ok(calls.some(c => c[0] === 'fill' && c[1] === 'evenodd'));
  assert.ok(calls.some(c => c[0] === 'moveTo' && c[1] === 40 && c[2] === -40));
  calls.length = 0;
  sandbox.setMapBackground('moved-area');
  sandbox.drawMapBackground({...map, lon_min: 2, lon_max: 12});
  // Moving the map extent must not stretch or translate the island coordinates.
  assert.ok(calls.some(c => c[0] === 'moveTo' && c[1] === 40 && c[2] === -40));
  calls.length = 0;
  images[0].onload();
  sandbox.drawMapBackground(map);
  assert.equal(calls.filter(c => c[0] === 'drawImage').length, 1);
  assert.equal(calls.filter(c => c[0] === 'fill').length, 0);
  console.log('PASS: JS syntax, failed-image fallback, island holes, moved bounds, calibrated-image recovery');
})().catch(error => { console.error(error); process.exitCode = 1; });
