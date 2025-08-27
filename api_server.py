from flask import Flask, jsonify, request, send_from_directory
from flask import Response
from math import radians, sin, cos, atan2, sqrt
from time import time
import os
import json
import threading
import queue

app = Flask(__name__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# SSE listeners for real-time telemetry
_telemetry_listeners = set()  # set[queue.Queue]

def _broadcast_telemetry(payload):
    to_remove = []
    for q in list(_telemetry_listeners):
        try:
            # Drop stale items to prefer fresh telemetry
            while q.full():
                q.get_nowait()
            q.put_nowait(payload)
        except Exception:
            to_remove.append(q)
    for q in to_remove:
        _telemetry_listeners.discard(q)

state = {
    "drone": {
        "lat": 48.8566,
        "lng": 2.3522,
        "altitude": 120.0,
        "battery": 92.0,
        "voltage": None,
        "heading": 0.0,
        "mode": "STANDBY",
        "armed": False,
        "gps": {"sats": None, "fix": None, "hdop": None},
        "last_update": time()
    },
    "mission": {
        "destination": None,  # {lat,lng}
        "distance_m": None,
        "eta_min": None,
        "state": "IDLE",  # IDLE, PRE-FLIGHT, ARMED, ACTIVE, COMPLETE
        "verified": False
    }
}

SPEED_MPS = 12.5

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    d_lat = radians(lat2 - lat1)
    d_lon = radians(lon2 - lon1)
    a = sin(d_lat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(d_lon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

# Utilities
def pick(d, keys):
    for k in keys:
        if k in d:
            return d[k]

def as_float(v):
    try:
        return float(v) if v is not None else None
    except (ValueError, TypeError):
        return None

def estimate_battery_pct_from_voltage(voltage_v, cells=3):
    """Estimate LiPo percentage from pack voltage for a 3S by default.
    Uses a simple piecewise-linear curve per cell (OCV under light load).
    Returns a float in [0, 100]."""
    try:
        if voltage_v is None or cells is None or cells <= 0:
            return None
        per_cell = float(voltage_v) / float(cells)
        # (Voltage per cell, Percent)
        points = [
            (3.30, 0.0),
            (3.50, 10.0),
            (3.61, 20.0),
            (3.67, 30.0),
            (3.73, 40.0),
            (3.80, 50.0),
            (3.87, 60.0),
            (3.95, 70.0),
            (4.00, 80.0),
            (4.10, 90.0),
            (4.20, 100.0),
        ]
        if per_cell <= points[0][0]:
            return 0.0
        if per_cell >= points[-1][0]:
            return 100.0
        for i in range(1, len(points)):
            v0, p0 = points[i-1]
            v1, p1 = points[i]
            if per_cell <= v1:
                t = (per_cell - v0) / (v1 - v0)
                pct = p0 + t * (p1 - p0)
                if pct < 0.0: pct = 0.0
                if pct > 100.0: pct = 100.0
                return pct
    except Exception:
        return None
    return None

# Static file serving
@app.get('/')
def root():
    return send_from_directory(BASE_DIR, 'index.html')

@app.get('/assets/<path:filename>')
def assets_file(filename):
    return send_from_directory(os.path.join(BASE_DIR, 'assets'), filename)

@app.get('/api/health')
def health():
    return jsonify({"ok": True})

@app.get('/api/telemetry/stream')
def telemetry_stream():
    def gen():
        q = queue.Queue(maxsize=64)
        _telemetry_listeners.add(q)
        try:
            # Send initial snapshot
            yield f"data: {json.dumps(state['drone'])}\n\n"
            while True:
                try:
                    payload = q.get(timeout=15)
                    yield f"data: {json.dumps(payload)}\n\n"
                except queue.Empty:
                    # Keep-alive comment to prevent proxies from closing the connection
                    yield ": keep-alive\n\n"
        finally:
            _telemetry_listeners.discard(q)

    headers = {
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no"
    }
    return Response(gen(), headers=headers, mimetype="text/event-stream")

@app.get('/api/telemetry')
def telemetry():
    return jsonify(state['drone'])

@app.post('/api/telemetry')
def ingest_telemetry():
    data = request.get_json(force=True, silent=True) or {}
    d = state['drone']
    lat = pick(data, ['lat', 'latitude'])
    lng = pick(data, ['lng', 'lon', 'longitude'])
    alt = pick(data, ['alt', 'altitude', 'relative_alt'])
    heading = pick(data, ['heading', 'yaw'])
    battery = pick(data, ['battery', 'battery_pct', 'battery_percent', 'batteryPercent'])
    voltage = pick(data, ['voltage', 'vbatt', 'vbat', 'battery_voltage'])
    mode = pick(data, ['mode', 'flight_mode', 'apmode'])
    armed = pick(data, ['armed', 'is_armed'])

    if isinstance(battery, dict):
        battery = pick(battery, ['percent', 'pct'])

    lat_f = as_float(lat)
    lng_f = as_float(lng)
    alt_f = as_float(alt)
    hdg_f = as_float(heading)
    bat_f = as_float(battery)
    v_f = as_float(voltage)

    if lat_f is not None:
        d['lat'] = lat_f
    if lng_f is not None:
        d['lng'] = lng_f
    if alt_f is not None:
        d['altitude'] = alt_f
    if hdg_f is not None:
        d['heading'] = hdg_f
    if v_f is not None:
        d['voltage'] = v_f
    if bat_f is not None:
        d['battery'] = bat_f
    elif v_f is not None:
        est = estimate_battery_pct_from_voltage(v_f, cells=3)
        if est is not None:
            d['battery'] = est
    if mode is not None:
        d['mode'] = str(mode)
    if armed is not None:
        d['armed'] = bool(armed)

    # GPS info
    gps = data.get('gps') if isinstance(data.get('gps'), dict) else {}
    sats = pick(gps, ['satellites', 'sats', 'num_sats']) if gps else pick(data, ['satellites', 'sats', 'num_sats'])
    fix = pick(gps, ['fix', 'fix_type']) if gps else pick(data, ['fix', 'fix_type'])
    hdop = pick(gps, ['hdop']) if gps else pick(data, ['hdop'])
    if sats is not None or fix is not None or hdop is not None:
        d['gps'] = {
            'sats': as_float(sats) if as_float(sats) is not None else sats,
            'fix': as_float(fix) if as_float(fix) is not None else fix,
            'hdop': as_float(hdop) if as_float(hdop) is not None else hdop,
        }

    d['last_update'] = time()
    _broadcast_telemetry(d)
    return jsonify({'ok': True})


@app.get('/api/mission')
def mission():
    return jsonify(state['mission'])


@app.post('/api/mission/destination')
def set_destination():
    if state['mission']['state'] in ("ACTIVE", "PRE-FLIGHT", "ARMED"):
        return jsonify({"error": "Mission en cours"}), 400
    data = request.get_json(force=True)
    lat = data.get('lat')
    lng = data.get('lng')
    if lat is None or lng is None:
        return jsonify({"error": "Coordonnées manquantes"}), 400
    # Limit 1 km radius
    d = haversine(state['drone']['lat'], state['drone']['lng'], lat, lng)
    if d > 1000:
        return jsonify({"error": "Destination hors rayon 1 km"}), 400
    m = state['mission']
    m['destination'] = {"lat": lat, "lng": lng}
    m['verified'] = False
    m['distance_m'] = None
    m['eta_min'] = None
    m['state'] = 'IDLE'
    return jsonify({"ok": True, "destination": m['destination']})


@app.post('/api/mission/verify')
def verify():
    m = state['mission']
    if not m['destination']:
        return jsonify({"error": "Pas de destination"}), 400
    m['state'] = 'PRE-FLIGHT'
    d = haversine(state['drone']['lat'], state['drone']['lng'], m['destination']['lat'], m['destination']['lng'])
    m['distance_m'] = d
    m['eta_min'] = d / SPEED_MPS / 60
    m['verified'] = True
    return jsonify({"ok": True, "distance_m": d, "eta_min": m['eta_min']})


@app.post('/api/mission/launch')
def launch():
    m = state['mission']
    if not m['verified']:
        return jsonify({"error": "Itinéraire non vérifié"}), 400
    m['state'] = 'ACTIVE'
    return jsonify({"ok": True, "state": m['state']})


@app.post('/api/mission/reset')
def reset():
    state['mission'] = {
        "destination": None,
        "distance_m": None,
        "eta_min": None,
        "state": "IDLE",
        "verified": False
    }
    return jsonify({"ok": True})


@app.post('/api/mission/complete')
def mission_complete():
    m = state['mission']
    m['state'] = 'COMPLETE'
    return jsonify({"ok": True, "state": m['state']})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False, use_reloader=False)
