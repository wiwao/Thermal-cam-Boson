import os
import subprocess
import time

path='/boot/aID.txt'
path_n='/home/pi/damy.txt'
path_w='/etc/wpa_supplicant/wpa_supplicant.conf'
try:
	f=open(path)
	l=f.readlines()
	id1='ssid=\"'+''.join(l[0].splitlines())+'\"'
	pass1='psk=\"'+''.join(l[1].splitlines())+'\"'
	id2=''.join(l[0].splitlines())
        pass2=''.join(l[1].splitlines())
	f.close()
except IOError:
	id1='ssid="ABiPhone"'
	pass1='psk="1245678"'
	id2="ABiPhone"
        pass2="1245678"
print "Set iPhone's",id1,pass1,"!!"
f=open(path_n)
l1=f.readlines()
idn='ssid=\"'+''.join(l1[0].splitlines())+'\"'
passn='psk=\"'+''.join(l1[1].splitlines())+'\"'
f.close()
if id1 != idn or pass1 != passn :
    str1='ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\n'
    str2='update_config=1\n'
    str3='country=JP\n\n'
    str4='network={\n'
    str5='key_mgmt=WPA-PSK\n'
    str51='proto=RSN\n'
    str6='}\n'
    str_1=str1+str2+str3+str4+'	'+id1+'\n'+'	'+pass1+'\n'+str5+str6
    with open(path_w, mode='w') as f:
        f.write(str_1)
        f.close()
    with open(path_n, mode='w') as f1:
        str_2=id2+'\n'+pass2+'\n'
        f1.write(str_2)
        f1.close()
    os.system('sudo reboot now')
#os.system('sh start2.sh &')
r=1
time.sleep(5)
while r>=1:
    print " Start!!"
    r=subprocess.call(["ping","-c","2","172.20.10.1"])
    print "Naw try!!"
    rx=subprocess.call(["sudo","ifconfig","wlan0","up"])
    time.sleep(4)
