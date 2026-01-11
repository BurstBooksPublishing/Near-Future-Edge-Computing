#!/usr/bin/env python3
import subprocess, json, time, logging, sys

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')

def run_iperf3(server, duration=10, parallel=4, port=5201, retry=3):
    cmd = [
        'iperf3', '--client', server, '--json',
        '--time', str(duration), '--parallel', str(parallel),
        '--port', str(port)
    ]
    for attempt in range(1, retry+1):
        logging.info('iperf3 attempt %d to %s', attempt, server)
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if proc.returncode == 0:
            try:
                return json.loads(proc.stdout)
            except json.JSONDecodeError:
                logging.warning('bad json, attempt %d', attempt)
        else:
            logging.warning('iperf3 failed: %s', proc.stderr.strip())
        time.sleep(1.0)
    raise RuntimeError('iperf3 failed after retries')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: iperf_client.py  [duration] [parallel]')
        sys.exit(2)
    server = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    parallel = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    result = run_iperf3(server, duration, parallel)
    # extract useful metrics and print stable values for CI systems
    bits_per_second = result['end']['sum_sent']['bits_per_second']
    retransmits = result['end']['sum_sent'].get('retransmits', 0)
    logging.info('throughput_bps=%d retransmits=%d', bits_per_second, retransmits)
    print(json.dumps({
        'throughput_bps': bits_per_second,
        'retransmits': retransmits,
        'timestamp': int(time.time())
    }))