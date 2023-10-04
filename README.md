# RCOM Project 1

This repository holds the source code of the first project in the RCOM curricular unit (Computer Networks).

The objective of this project is to send a file (via serial port) between two different machines


### Commands
Create virtual Serial Port (just Linux): 
```
sudo socat -d  -d  PTY,link=/dev/ttyS10,mode=777   PTY,link=/dev/ttyS11,mode=777
```
To compile all project:
```
make
```
To run receiver:
```
make run_receiver
```

To run sender:
```
make run_sender
```
To clean binary files:
```
make clean
```
