#!/usr/bin/env python3
"""UDP send test — run on simulator (.22). Target RV2 by default."""
import socket, sys

target = sys.argv[1] if len(sys.argv) > 1 else '192.168.100.20'
port = int(sys.argv[2]) if len(sys.argv) > 2 else 14550
n = int(sys.argv[3]) if len(sys.argv) > 3 else 500

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(n):
    s.sendto(f'pkt{i}'.encode(), (target, port))
print(f'sent {n} packets to {target}:{port}')
s.close()
