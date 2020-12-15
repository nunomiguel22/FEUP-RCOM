#!/bin/bash

service networking restart

ifconfig eth0 172.16.60.1/24
route add default gw 172.16.60.254

echo 1 > /proc/sys/net/ipv4/ip_forward
echo 0 > /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts
