from __future__ import print_function
from asyncore import write
# from sys import stdout
import time
# from unittest import result
# import serial
import subprocess
# import re

def find_usb():
    global usb
    usb =''
    proc = subprocess.Popen(["lsblk"], stdout=subprocess.PIPE, shell=True)
    (out_usb,error) = proc.communicate()
    out_usb_new = out_usb.decode().split('\n')
    for i in range(0,len(out_usb_new)):
        out_data =out_usb_new[i].strip().split()
        for n in range(0,len(out_data)):
            if '/media/pi/' in out_data[n]:
                usb = out_data[n]
                usb = usb+'/speedTestFile'
                usb = usb.strip()
                break
    return usb
# coypy_file_cmd = ("sudo","sshpass","-p","gremsy1304","scp","/home/pi/CV25_carrier_test_jig/document/CV25_Mio_tester","root@192.168.30.11:/root")
# time.sleep(0.5)
# proc = subprocess.check_output(coypy_file_cmd)

def check_speed_write_usb(usb):
    speed_write=0
    # mount vào usb
    cmd_check_usb = 'sudo dd if=/dev/zero of={} bs=20M count=5 oflag=direct'.format(usb)
    print(cmd_check_usb)
    proc = subprocess.Popen([cmd_check_usb], stderr=subprocess.PIPE, shell=True)
    time.sleep(0.1)
    out = proc.stderr.read()
    # print('out_raw:',out)
    out_new = out.decode().split('\n')
    # print(type(out_new))
    # print('output:',out_new)
    for i in range(0,len(out_new)):
        out_speed =out_new[i].strip().split(',')
        # print(out_speed)
        for n in range(0,len(out_speed)):
            if 'MB/s' in out_speed[n]:
                print('speed_write:',out_speed[n])
                speed_write = out_speed[n]
                break
    return speed_write
def check_speed_read_usb(usb):
    speed_read = 0
    cmd_check_read_usb ="sudo dd if={} of=/dev/zero bs=20M count=5 oflag=dsync".format(usb)
    print(cmd_check_read_usb)
    #  mount vào usb
    proc = subprocess.Popen([cmd_check_read_usb], stderr=subprocess.PIPE, shell=True)
    time.sleep(0.1)
    out = proc.stderr.read()
    # print('out_raw:',out)
    out_new = out.decode().split('\n')
    # print(type(out_new))
    # print('output:',out_new)
    for i in range(0,len(out_new)):
        out_speed =out_new[i].strip().split(',')
        # print(out_speed)
        for n in range(0,len(out_speed)):
            if 'MB/s' in out_speed[n]:
                print('speed_read:',out_speed[n])
                speed_read = out_speed[n]
                break
    return speed_read

# check pass fail
def check_speed_usb():
    write_PF = False
    read_PF = False
    result = False
    # find usb
    name_usb = find_usb()
    if len(name_usb)!= 14:
        # check speed write
        write_speed = check_speed_write_usb(name_usb)
        if write_speed != 0:
            write_speed = write_speed.split(' ')
            print(write_speed[1])
            if float(write_speed[1]) >= 20:  # spec write > 20 MBps
                write_PF = True

        # check speed read
        read_speed = check_speed_read_usb(name_usb)
        if read_speed != 0 :
            read_speed = read_speed.split(' ')
            print(read_speed[1])
            if float(read_speed[1]) >=20 :   # spec read > 20 MBps
                read_PF = True
                
    name_delete = "rm -f {}".format(name_usb)
    umount_usb = "sudo umount /dev/sdb1"
    proc = subprocess.Popen([name_delete], stderr=subprocess.PIPE, shell=True)
    time.sleep(0.1)
    proc = subprocess.Popen([umount_usb], stderr=subprocess.PIPE, shell=True)
    time.sleep(0.1)
    # output result
    if write_PF == True and read_PF == True:
        result = True
    
    return result
