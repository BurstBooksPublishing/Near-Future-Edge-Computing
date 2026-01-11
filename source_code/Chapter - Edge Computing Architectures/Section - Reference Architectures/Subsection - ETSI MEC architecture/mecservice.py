from flask import Flask, request, jsonify, abort
import os, logging, jwt, time

# production: configure logging, TLS termination, gunicorn workers
logging.basicConfig(level=logging.INFO, format='%(message)s')
app = Flask(__name__)
SECRET = os.environ.get('MEC_JWT_SECRET', 'changeme')  # use secret in k/v store

def authorize(req):
    auth = req.headers.get('Authorization')
    if not auth:
        return False
    try:
        token = auth.split()[1]
        jwt.decode(token, SECRET, algorithms=['HS256'])
        return True
    except Exception:
        return False

@app.route('/location/v1/ue/', methods=['GET'])
def location(ue_id):
    if not authorize(request):
        abort(401)
    # production: query RAN controller or NEF for real-time UE coords
    sample = {'ue_id': ue_id, 'lat': 52.0, 'lon': 4.0, 'ts': time.time()}
    logging.info({'event':'location_query','ue':ue_id})
    return jsonify(sample)

@app.route('/rnis/v1/ue/', methods=['GET'])
def rnis(ue_id):
    if not authorize(request):
        abort(401)
    # production: return measured RSRP/RSRQ/QoS reported by RAN; here synthetic
    sample = {'ue_id': ue_id, 'rsrp_dbm': -75, 'rsrq_db': -10, 'bandwidth_mbps': 50}
    logging.info({'event':'rnis_query','ue':ue_id})
    return jsonify(sample)

if __name__ == '__main__':
    # For development only. In production, run via Gunicorn and enable TLS.
    app.run(host='0.0.0.0', port=8080)