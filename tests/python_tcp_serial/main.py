#!/usr/bin/env python

import socket

from pprint import pprint

TCP_IP = '127.0.0.1'
TCP_PORT = 2390
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
# s.send(MESSAGE)

while True:
  data = s.recv(BUFFER_SIZE)
  if not data: break
  pprint(data)


s.close()
