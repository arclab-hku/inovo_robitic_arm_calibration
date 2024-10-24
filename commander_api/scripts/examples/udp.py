import socket
import binascii
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# addr = ("192.168.8.116", 3333)
# addr = ("192.168.137.239", 3333)
# addr = ("10.70.24.91", 3333)
addr = ("192.168.8.126", 3333)

print("Socket connected")

data = b'\x00\x60\x00'#open
# data = b'\x00\x35\x00'#open (original?)
# data = b'\x00\x00\x01'#close
# data = b'\x00\x00\x02'#stop


s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print(response)

time.sleep(10)

# data = b'\x00\x35\x00' #open (Add speed? how to adjust the speed?)
# data = b'\x00\x00\x01' #close
# data = b'\x00\x00\x02' #add force
data = b'\x00\x00\x03' #stop


# print(data)
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print(response)


s.close()