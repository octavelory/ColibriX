from flask import Flask, jsonify, request, send_from_directory
from math import radians, sin, cos, atan2, sqrt
from time import time
import os

app = Flask(__name__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

state = {
    "drone": {
        "lat": 48.8566,
        "lng": 2.3522,
        "altitude": 120.0,
        "battery": 92.0,
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
    mode = pick(data, ['mode', 'flight_mode', 'apmode'])
    armed = pick(data, ['armed', 'is_armed'])

    if isinstance(battery, dict):
        battery = pick(battery, ['percent', 'pct'])

    lat_f = as_float(lat)
    lng_f = as_float(lng)
    alt_f = as_float(alt)
    hdg_f = as_float(heading)
    bat_f = as_float(battery)

    if lat_f is not None:
        d['lat'] = lat_f
    if lng_f is not None:
        d['lng'] = lng_f
    if alt_f is not None:
        d['altitude'] = alt_f
    if hdg_f is not None:
        d['heading'] = hdg_f
    if bat_f is not None:
        d['battery'] = bat_f
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
    app.run(host='0.0.0.0', port=5000, debug=True)
