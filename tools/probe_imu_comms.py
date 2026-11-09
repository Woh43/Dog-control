#!/usr/bin/env python3
"""Receive-only FDILink serial capture, CRC validation; never sends bytes."""
import binascii,collections,json,math,os,select,struct,termios,time,fcntl
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port',default='/dev/ttyUSB0')
parser.add_argument('--duration',type=float,default=5.0)
parser.add_argument('--output',required=True)
args=parser.parse_args()
if not 0.5 <= args.duration <= 30:parser.error('duration must be 0.5..30 seconds')
PORT=args.port;DURATION=args.duration
fd=os.open(PORT,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
fcntl.ioctl(fd,termios.TIOCEXCL)
original=termios.tcgetattr(fd)
config=termios.tcgetattr(fd); config[0]=config[1]=config[3]=0
config[2]=termios.CS8|termios.CREAD|termios.CLOCAL
config[4]=config[5]=termios.B921600
config[6][termios.VMIN]=0;config[6][termios.VTIME]=0
termios.tcsetattr(fd,termios.TCSANOW,config)
def crc8(data):
    value=0
    for byte in data:
        value^=byte
        for _ in range(8):value=(value>>1)^(0x8C if value&1 else 0)
    return value
buffer=bytearray();counts=collections.Counter();bad=collections.Counter();payloads={};stamps={};raw=bytearray();start=time.monotonic()
try:
    while time.monotonic()-start<DURATION:
        if select.select([fd],[],[],.05)[0]:
            data=os.read(fd,8192);raw.extend(data);buffer.extend(data)
        while len(buffer)>=7:
            if buffer[0]!=0xfc: del buffer[0];bad['discarded_bytes']+=1;continue
            if crc8(buffer[:4])!=buffer[4]:del buffer[0];bad['header_crc']+=1;continue
            length=buffer[2]+8
            if len(buffer)<length:break
            frame=bytes(buffer[:length]);del buffer[:length]
            if frame[-1]!=0xfd:bad['end_marker']+=1;continue
            payload=frame[7:-1]
            if binascii.crc_hqx(payload,0)!=(frame[5]<<8|frame[6]):bad['payload_crc']+=1;continue
            kind=hex(frame[1]);counts[kind]+=1;payloads[kind]=payload;stamps.setdefault(kind,[]).append(time.monotonic())
finally:
    termios.tcsetattr(fd,termios.TCSANOW,original);fcntl.ioctl(fd,termios.TIOCNXCL);os.close(fd)
result={'port':PORT,'baud':921600,'duration_s':time.monotonic()-start,'receive_only':True,'bytes':len(raw),'valid_frames':dict(counts),'errors':dict(bad),'rate_hz':{k:(len(v)-1)/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>1}}
if '0x40' in payloads:
    values=struct.unpack('<12fq',payloads['0x40']); result['last_imu']={'gyro_rad_s':values[:3],'accel_m_s2':values[3:6],'accel_norm':math.sqrt(sum(x*x for x in values[3:6])),'temperature_c':values[9],'timestamp_us':values[12]}
if '0x41' in payloads:
    values=struct.unpack('<10fq',payloads['0x41']);result['last_ahrs']={'rpy_rad':values[3:6],'quaternion_wxyz':values[6:10],'quaternion_norm':math.sqrt(sum(x*x for x in values[6:10])),'timestamp_us':values[10]}
open(args.output+'.bin','wb').write(raw)
open(args.output,'w').write(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))

# Lack of usable frames must fail a diagnostic invocation.
import sys
valid=(counts.get('0x40',0)>0 and counts.get('0x41',0)>0 and
       not bad.get('header_crc',0) and not bad.get('payload_crc',0) and
       abs(result.get('last_ahrs',{}).get('quaternion_norm',0)-1)<0.01)
sys.exit(0 if valid else 2)
