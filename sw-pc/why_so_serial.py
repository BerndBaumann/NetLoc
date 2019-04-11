#!/usr/bin/env python3
import threading
import sys
import serial
import io
import time
from datetime import datetime

keep_alive_ = True
include_timestamp_ = False

def waitingForReset(ser, sio):
    global keep_alive_
    l = ""
    while keep_alive_ and "Contiki" not in l:
        try:
            l = sio.readline()
        except UnicodeDecodeError as ude:
            print(".")
            pass
    return l

def writeLogFile(ser, filename=None):
    global keep_alive_
    global include_timestamp_
    try:
        sio = io.TextIOWrapper(io.BufferedReader(ser))
        firstline = waitingForReset(ser, sio)
        print(firstline)
        fn = firstline.lstrip("Contiki on node ")
        if filename == None:
            filename = ''
            for c in fn.split():
                filename += c
            filename += ".log"
        # dump all serial output into a file
        # until the user enters: 'q <Enter>'
        # If the "Contiki on node ..." line is read
        # again, overwrite the existing file and
        # start over.
        # time.sleep(0) is equivalent to yield()
        while keep_alive_:
            l = ""
            with open(filename, "w") as logfile:
                logfile.write(firstline)
                while keep_alive_ and "Contiki" not in l:
                    l = sio.readline()
                    if include_timestamp_ and len(l) > 2:
                        now = datetime.now()
                        ts = "({0:>4d}-{1:>02d}-{2:>02d} {3:>02d}:{4:>02d}:{5:>02d}) ".format(
                            now.year, now.month, now.day, now.hour, now.minute, now.second)
                        logfile.write(ts)
                        sys.stdout.write(ts)
                    logfile.write(l)
                    logfile.flush()
                    sys.stdout.write(l)
                    time.sleep(0)
    except serial.serialutil.SerialException as se:
        sys.stderr.write("Disconnected\n")
        sys.exit(1)

def readCommand():
    global keep_alive_
    global include_timestamp_
    abort = "q"
    timestamp = "t"
    while True:
        cmd = sys.stdin.readline()
        if abort in cmd:
            keep_alive_ = False
            break
        if timestamp in cmd:
            include_timestamp_ = not include_timestamp_
            print("Adding timestamps")
        time.sleep(0)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: {} <port> [speed]".format(sys.argv[0]))
        print("  to open a serial connection on port /dev/ttyACM<port>")
        print("  the default speed is 921600")
        print("\n  press 't <ENTER>' to toggle timestamps")
        print("\n  press 'q <ENTER>' to EXIT")
        sys.exit()
    
    baudrate = 921600
    
    if len(sys.argv) == 3:
        baudrate = int(sys.argv[2])
    print("Opening port /dev/ttyACM{} at {}".format(sys.argv[1], baudrate))
    
    ser = serial.Serial()
    ser.port = "/dev/ttyACM{}".format(sys.argv[1])
    ser.baudrate = baudrate
    ser.xonxoff = 0
    ser.rtscts = 0
    ser.bytesize = 8
    ser.parity = 'N'
    ser.timeout = 1
    ser.stopbits = 1
    
    ser.open()
    
    writer = threading.Thread(target = writeLogFile, name = "WriteLogFile", args = (ser,))
    reader = threading.Thread(target = readCommand, name = "ReadCommand")
    
    writer.start()
    reader.start()
    
    writer.join()
    
    ser.close()
