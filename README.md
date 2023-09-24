# RCOM Project 1

This repository holds the source code of the first project in the RCOM curricular unit (Computer Networks).

The objective of this project is to send a file (via serial port) between two different machines

Some commands: 
```
sudo socat -d  -d  PTY,link=/dev/ttyS10,mode=777   PTY,link=/dev/ttyS11,mode=777
gcc write_noncanonical.c -o write
gcc read_noncanonical.c -o read
./write /dev/ttyS11
./read /dev/ttyS10
```
