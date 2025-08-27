// ColibriX Mission Console Frontend
// Futuristic UI logic with mock backend simulation (now adaptable to API)

const AppState = {
  drone: {
    lat: 48.8566,
    lng: 2.3522,
    altitude: 120,
    battery: 92,
    heading: 0,
    lastUpdate: Date.now()
  },
  mission: {
    destination: null,
    distanceMeters: null,
    etaMinutes: null,
    state: 'IDLE',
    verified: false
  },
  map: null,
  layer: {
    droneMarker: null,
    destMarker: null,
    radiusCircle: null,
    routeLine: null
  }
};

const kmRadius = 1.0; // 1 km selection radius

// API client skeleton (customize baseURL for real backend)
const API = {
  baseURL: '/api',
  async telemetry() { return fetch(this.baseURL + '/telemetry').then(r=>r.json()); },
  async mission() { return fetch(this.baseURL + '/mission').then(r=>r.json()); },
  async setDestination(lat,lng) { return fetch(this.baseURL + '/mission/destination', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({lat,lng})}).then(r=>r.json()); },
  async verify() { return fetch(this.baseURL + '/mission/verify', {method:'POST'}).then(r=>r.json()); },
  async launch() { return fetch(this.baseURL + '/mission/launch', {method:'POST'}).then(r=>r.json()); },
  async reset() { return fetch(this.baseURL + '/mission/reset', {method:'POST'}).then(r=>r.json()); }
};

// SSE telemetry stream state
const TelemetryStream = { es: null, connected: false, lastMsg: 0 };

function log(message, level='info') {
  const stream = document.getElementById('log-stream');
  const time = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.className = 'log-entry ' + (level==='ok'?'ok':level==='warn'?'warn':level==='err'?'err':'');
  div.innerHTML = `<span class="time">${time}</span>${message}`;
  stream.appendChild(div);
  stream.scrollTop = stream.scrollHeight;
}

// Notifications removed per request

function updateHudClock() {
  document.getElementById('hud-time').textContent = new Date().toLocaleTimeString();
}

function updateStatusPanel() {
  document.getElementById('stat-lat').textContent = AppState.drone.lat.toFixed(5);
  document.getElementById('stat-lng').textContent = AppState.drone.lng.toFixed(5);
  document.getElementById('stat-alt').textContent = AppState.drone.altitude.toFixed(0) + ' m';
  document.getElementById('stat-bat').textContent = AppState.drone.battery.toFixed(0) + '%';
  const missionState = document.getElementById('mission-state');
  missionState.textContent = translateMissionState(AppState.mission.state);
  missionState.className = 'badge ' + AppState.mission.state.replace(/[^A-Z]/g,'').toLowerCase();
}

function applyTelemetry(t) {
  if (!t) return;
  if (typeof t.lat === 'number') AppState.drone.lat = t.lat;
  if (typeof t.lng === 'number') AppState.drone.lng = t.lng;
  if (typeof t.altitude === 'number') AppState.drone.altitude = t.altitude;
  if (typeof t.battery === 'number') AppState.drone.battery = t.battery;
  if (typeof t.heading === 'number') AppState.drone.heading = t.heading;
  if (t.gps) {
    const gpsEl = document.getElementById('hud-gps');
    const sats = (t.gps.sats ?? '--');
    const fix = (t.gps.fix ?? '--');
    gpsEl.textContent = `GPS : ${sats} sat, fix ${fix}`;
  }
  updateStatusPanel();
  updateDroneMarker();
}

function translateMissionState(s) {
  switch(s) {
    case 'IDLE': return 'EN ATTENTE';
    case 'PRE-FLIGHT': return 'PRÉVOL';
    case 'ARMED': return 'ARMÉ';
    case 'ACTIVE': return 'EN VOL';
    case 'COMPLETE': return 'TERMINÉE';
    default: return s;
  }
}

function initMap() {
  const map = L.map('map', {
    zoomControl:false,
    attributionControl: false,
    scrollWheelZoom: false,
    doubleClickZoom: false,
    touchZoom: true
  }).setView([AppState.drone.lat, AppState.drone.lng], 15);
  AppState.map = map;

  L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 19 }).addTo(map);

  AppState.layer.droneMarker = L.marker([AppState.drone.lat, AppState.drone.lng], {
    icon: L.divIcon({
      className:'drone-ico',
      html:`<div class="drone-marker"><div class="pulse"></div><svg width="42" height="42" viewBox="0 0 52 52"><circle cx="26" cy="26" r="10" stroke="#15f4ff" stroke-width="2" fill="rgba(21,244,255,0.15)"/><path d="M26 2 L32 14 L26 12 L20 14 Z" fill="#15f4ff" opacity="0.6"/><circle cx="26" cy="26" r="3" fill="#15f4ff"/></svg></div>`
    })
  }).addTo(map);

  AppState.layer.radiusCircle = L.circle([AppState.drone.lat, AppState.drone.lng], {
    radius: kmRadius*1000,
    color:'#15f4ff',
    weight:1.2,
    fillColor:'#15f4ff',
    fillOpacity:0.05
  }).addTo(map);

  map.on('click', handleMapClick);
}

function handleMapClick(e) {
  if (['ACTIVE','PRE-FLIGHT','ARMED'].includes(AppState.mission.state)) return;
  const d = distanceMeters(AppState.drone.lat, AppState.drone.lng, e.latlng.lat, e.latlng.lng);
  if (d > kmRadius*1000) return;
  setDestination(e.latlng);
}

async function setDestination(latlng) {
  try {
    const resp = await API.setDestination(latlng.lat, latlng.lng);
    if (resp && resp.error) { log(`Erreur destination: ${resp.error}`, 'err'); return; }
    const dest = (resp && resp.destination) ? resp.destination : { lat: latlng.lat, lng: latlng.lng };
    AppState.mission.destination = dest;
    document.getElementById('dest-coords').textContent = `${dest.lat.toFixed(5)}, ${dest.lng.toFixed(5)}`;
    enableButton('btn-verify', true);
    enableButton('btn-reset', true);
    if (AppState.layer.destMarker) AppState.map.removeLayer(AppState.layer.destMarker);
    if (AppState.layer.routeLine) { AppState.map.removeLayer(AppState.layer.routeLine); AppState.layer.routeLine=null; }
    AppState.mission.verified = false;
    enableButton('btn-launch', false);
    document.getElementById('route-info').classList.add('hidden');
    AppState.layer.destMarker = L.marker([dest.lat, dest.lng], {
      icon: L.divIcon({
        className:'dest-ico',
        iconSize:[42,42],
        iconAnchor:[21,21],
        html:`<div class="dest-marker"><div class="ring"></div><div class="core"></div></div>`
      })
    }).addTo(AppState.map);
    log(`Destination définie : ${dest.lat.toFixed(5)}, ${dest.lng.toFixed(5)}`);
  } catch (err) {
    log('Erreur réseau lors de la définition de la destination', 'err');
  }
}

function enableButton(id, enabled) {
  const btn = document.getElementById(id);
  if (enabled) btn.removeAttribute('disabled'); else btn.setAttribute('disabled','true');
}

function distanceMeters(lat1,lng1,lat2,lng2) {
  const R = 6371000;
  const toRad = x => x * Math.PI/180;
  const dLat = toRad(lat2-lat1);
  const dLng = toRad(lng2-lng1);
  const a = Math.sin(dLat/2)**2 + Math.cos(toRad(lat1))*Math.cos(toRad(lat2))*Math.sin(dLng/2)**2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R * c;
}

async function verifyRoute() {
  if (!AppState.mission.destination) return;
  showOverlay('Vérification','Analyse du trajet...');
  log('Vérification de l\'itinéraire');
  try {
    const resp = await API.verify();
    hideOverlay();
    if (resp && resp.error) { log(`Erreur vérification: ${resp.error}`, 'err'); return; }
    AppState.mission.state = 'PRE-FLIGHT';
    AppState.mission.distanceMeters = resp.distance_m;
    AppState.mission.etaMinutes = resp.eta_min;
    AppState.mission.verified = true;
    drawRoute();
    const distKm = (resp.distance_m/1000).toFixed(2);
    document.getElementById('route-distance').textContent = distKm + ' km';
    document.getElementById('route-eta').textContent = AppState.mission.etaMinutes.toFixed(1) + ' min';
    document.getElementById('route-info').classList.remove('hidden');
    enableButton('btn-launch', true);
    updateStatusPanel();
    log('Itinéraire vérifié');
  } catch (e) {
    hideOverlay();
    log('Erreur réseau lors de la vérification', 'err');
  }
}

function drawRoute() {
  if (AppState.layer.routeLine) AppState.map.removeLayer(AppState.layer.routeLine);
  const p1 = [AppState.drone.lat, AppState.drone.lng];
  const p2 = [AppState.mission.destination.lat, AppState.mission.destination.lng];
  AppState.layer.routeLine = L.polyline([p1,p2], { color:'#15f4ff', weight:3, opacity:0.9, dashArray:'6 10' }).addTo(AppState.map);
  AppState.map.fitBounds(L.latLngBounds(p1,p2).pad(0.3));
}

async function launchMission() {
  if (!AppState.mission.verified) return;
  log('Armement de la mission...');
  showOverlay('Armement','Contrôles pré-vol');
  try {
    const resp = await API.launch();
    setTimeout(()=> { hideOverlay(); }, 1200);
    if (resp && resp.error) { log(`Erreur lancement: ${resp.error}`, 'err'); return; }
    AppState.mission.state = resp.state || 'ACTIVE';
    updateStatusPanel();
    log('Mission lancée');
  } catch (e) {
    hideOverlay();
    log('Erreur réseau lors du lancement', 'err');
  }
}

async function resetMission() {
  try { await API.reset(); } catch (e) { /* ignore */ }
  AppState.mission = { destination:null,distanceMeters:null,etaMinutes:null,state:'IDLE',verified:false };
  if (AppState.layer.destMarker) { AppState.map.removeLayer(AppState.layer.destMarker); AppState.layer.destMarker=null; }
  if (AppState.layer.routeLine) { AppState.map.removeLayer(AppState.layer.routeLine); AppState.layer.routeLine=null; }
  document.getElementById('dest-coords').textContent = 'Aucune';
  document.getElementById('route-info').classList.add('hidden');
  enableButton('btn-launch', false);
  enableButton('btn-verify', false);
  enableButton('btn-reset', false);
  updateStatusPanel();
  log('Mission réinitialisée');
}

function showOverlay(title, desc) {
  const overlay = document.getElementById('mission-overlay');
  document.getElementById('overlay-title').textContent = title;
  document.getElementById('overlay-desc').textContent = desc;
  overlay.classList.remove('hidden');
}
function hideOverlay() { document.getElementById('mission-overlay').classList.add('hidden'); }

async function pollBackend() {
  if (!TelemetryStream.connected) {
    try {
      const t = await API.telemetry();
      applyTelemetry(t);
    } catch (e) { /* silent */ }
  }

  try {
    const m = await API.mission();
    if (m) {
      const prevDest = AppState.mission.destination;
      if (m.state) AppState.mission.state = m.state;
      if (typeof m.verified !== 'undefined') AppState.mission.verified = !!m.verified;
      if (typeof m.distance_m === 'number') AppState.mission.distanceMeters = m.distance_m;
      if (typeof m.eta_min === 'number') AppState.mission.etaMinutes = m.eta_min;
      if (m.destination) AppState.mission.destination = m.destination;

      if (AppState.mission.destination && (!prevDest || prevDest.lat !== AppState.mission.destination.lat || prevDest.lng !== AppState.mission.destination.lng)) {
        document.getElementById('dest-coords').textContent = `${AppState.mission.destination.lat.toFixed(5)}, ${AppState.mission.destination.lng.toFixed(5)}`;
        if (AppState.layer.destMarker) AppState.map.removeLayer(AppState.layer.destMarker);
        AppState.layer.destMarker = L.marker([AppState.mission.destination.lat, AppState.mission.destination.lng], {
          icon: L.divIcon({ className:'dest-ico', iconSize:[42,42], iconAnchor:[21,21], html:`<div class=\"dest-marker\"><div class=\"ring\"></div><div class=\"core\"></div></div>` })
        }).addTo(AppState.map);
      }

      if (AppState.mission.distanceMeters && AppState.mission.etaMinutes) {
        document.getElementById('route-distance').textContent = (AppState.mission.distanceMeters/1000).toFixed(2) + ' km';
        document.getElementById('route-eta').textContent = AppState.mission.etaMinutes.toFixed(1) + ' min';
        document.getElementById('route-info').classList.remove('hidden');
      }

      enableButton('btn-verify', !!AppState.mission.destination && !AppState.mission.verified);
      enableButton('btn-launch', AppState.mission.verified && AppState.mission.state !== 'ACTIVE');
      enableButton('btn-reset', true);
    }
  } catch (e) { /* silent */ }

  updateStatusPanel();
  updateDroneMarker();
}

function initTelemetryStream() {
  try {
    const es = new EventSource('/api/telemetry/stream');
    TelemetryStream.es = es;
    es.onopen = () => { TelemetryStream.connected = true; TelemetryStream.lastMsg = Date.now(); log('Flux télémétrie connecté', 'ok'); };
    es.onmessage = (ev) => {
      TelemetryStream.lastMsg = Date.now();
      try { applyTelemetry(JSON.parse(ev.data)); } catch (_) { /* ignore parse errors */ }
    };
    es.onerror = () => {
      TelemetryStream.connected = false;
      try { es.close(); } catch (_) {}
      log('Flux télémétrie déconnecté, reconnexion...', 'warn');
      setTimeout(initTelemetryStream, 2000);
    };
  } catch (e) {
    // Fallback to polling only
  }
}

function updateDroneMarker() {
  if (AppState.layer.droneMarker) {
    AppState.layer.droneMarker.setLatLng([AppState.drone.lat, AppState.drone.lng]);
    AppState.layer.radiusCircle.setLatLng([AppState.drone.lat, AppState.drone.lng]);
    if (AppState.mission.state==='ACTIVE' && AppState.layer.routeLine && AppState.mission.destination) {
      drawRoute();
    }
  }
}

function completeMission() {
  AppState.mission.state = 'COMPLETE';
  updateStatusPanel();
  log('Mission terminée');
  enableButton('btn-reset', true);
  enableButton('btn-launch', false);
  enableButton('btn-verify', false);
}

function simulateFlight() { /* movement simulated in poll */ }

function bindUI() {
  document.getElementById('btn-verify').addEventListener('click', verifyRoute);
  document.getElementById('btn-launch').addEventListener('click', launchMission);
  document.getElementById('btn-reset').addEventListener('click', resetMission);
  const toggleLog = document.getElementById('toggle-log');
  toggleLog.addEventListener('click', () => {
    const stream = document.getElementById('log-stream');
    const expanded = toggleLog.getAttribute('aria-expanded') === 'true';
    if (expanded) { stream.style.display='none'; toggleLog.textContent='Afficher'; toggleLog.setAttribute('aria-expanded','false'); }
    else { stream.style.display='block'; toggleLog.textContent='Masquer'; toggleLog.setAttribute('aria-expanded','true'); }
  });
  const zoomInBtn = document.getElementById('zoom-in');
  const zoomOutBtn = document.getElementById('zoom-out');
  zoomInBtn.addEventListener('click', () => smoothZoom(1));
  zoomOutBtn.addEventListener('click', () => smoothZoom(-1));
  AppState.map.getContainer().addEventListener('wheel', (e)=> { if (e.ctrlKey) { e.preventDefault(); smoothZoom(e.deltaY < 0 ? 1 : -1); } }, { passive:false });
}

function smoothZoom(direction) {
  const map = AppState.map;
  const target = map.getZoom() + direction;
  if (target < 3 || target > 19) return;
  map.setZoom(target, { animate:true });
}

function initCustomStyles() {
  const style = document.createElement('style');
  style.textContent = `
  .drone-marker { position:relative; }
  .drone-marker .pulse { position:absolute; top:50%; left:50%; width:18px; height:18px; transform:translate(-50%,-50%); border:2px solid #15f4ff; border-radius:50%; animation: pulse 2.4s ease-out infinite; }
  @keyframes pulse { 0% { opacity:.9; transform:translate(-50%,-50%) scale(.25);} 70% { opacity:0; transform:translate(-50%,-50%) scale(1.8);} 100%{ opacity:0;} }
  .dest-marker { position:relative; width:42px; height:42px; }
  .dest-marker .ring { position:absolute; top:50%; left:50%; width:42px; height:42px; margin-left:-21px; margin-top:-21px; border:2px dashed #15f4ff; border-radius:50%; animation: spin 6s linear infinite; opacity:.65; }
  .dest-marker .core { position:absolute; top:50%; left:50%; width:14px; height:14px; background:#15f4ff; border-radius:50%; transform:translate(-50%,-50%); box-shadow:0 0 14px 2px #15f4ffaa; }
  `;
  document.head.appendChild(style);
}

function setupThemeToggle() {
  const btn = document.getElementById('theme-toggle');
  if (!btn) return;
  btn.addEventListener('click', ()=> {
    const light = document.body.classList.toggle('light');
    btn.textContent = light ? 'Thème Sombre' : 'Thème Clair';
  });
}

function bootstrap() {
  initMap();
  bindUI();
  initCustomStyles();
  setupThemeToggle();
  updateStatusPanel();
  setInterval(updateHudClock, 1000);
  initTelemetryStream();
  pollBackend();
  setInterval(pollBackend, 1000);
  log('Système initialisé');
}

document.addEventListener('DOMContentLoaded', bootstrap);
