#!/usr/bin/env python3
"""UDP receive test — run on RV2 (.20). Stop Docker first if testing port 14550."""
import socket, sys

port = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', port))
s.settimeout(12)
print(f'Listening on 0.0.0.0:{port} (12s timeout)...')
count = 0
try:
    while True:
        data, addr = s.recvfrom(1024)
        count += 1
except KeyboardInterrupt:
    pass
except socket.timeout:
    pass
print(f'received {count} packets')
s.close()
