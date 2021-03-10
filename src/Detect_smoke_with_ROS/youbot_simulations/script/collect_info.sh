#!/bin/bash

touch rosnode_list.txt
rosnode list -a > rosnode_list.txt

touch running_process_info.txt
ps aux | less > running_process_info.txt

touch installed_packages_apt.txt
apt list --installed > installed_packages_apt.txt

touch installed_packages_dpkg.txt
dpkg -l > installed_packages_dpkg.txt

touch general_system_info.txt
uname -a > general_system_info.txt

touch hardware_info.txt
sudo lshw > hardware_info.txt

touch memory_info.txt
sudo dmidecode --type memory > memory_info.txt

touch rosparam_complet_info.txt
rosparam get / > rosparam_complet_info.txt

touch network_info.txt
ifconfig > network_info.txt

touch rosservice_info.txt
rosservice list -n > rosservice_info.txt

touch cpu_info.txt
lscpu > cpu_info.txt

touch ntp_info.txt
cat /etc/ntp.conf > ntp_info.txt

touch roswtf_info.txt
roswtf > roswtf_info.txt

touch host_info.txt
hostname > host_info.txt
cat /etc/hosts > host_info.txt

touch rospackage_info.txt
rospack list > rospackage_info.txt

touch nmap_info.txt
nmap -sP 192.168.1.0/24 > nmap_info.txt
