#!/usr/bin/env python

import socket

from pprint import pprint

TCP_IP = '127.0.0.1'
TCP_PORT = 2391
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
# s.send(MESSAGE)

while True:
  # print('>>>>>>>>>>>>>>>>>')
  # s.send('0123456789')
  print('<<<<<<<<<<<<<<<<<')
  data = s.recv(BUFFER_SIZE)
  if not data: break
  for c in data:
    print "int: {0:d}; hex: {0:x}; bin: {0:b}; chr {1}".format(ord(c), c)


s.close()
