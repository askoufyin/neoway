#!/bin/sh
#
IIFACE="bridge0"
OIFACE=$1
IP=$2
GW=$3
DNS1=$4
DNS2=$5


echo "POST-CONNECT: sysctl"
sysctl -w net.ipv4.ip_forward=1

echo "POST-CONNECT: Configuring iptables"

# delete all existing rules.
iptables -F
iptables -t nat -F
iptables -t mangle -F
iptables -X

# Always accept loopback traffic
iptables -A INPUT -i lo -j ACCEPT

# Allow established connections, and those not coming from the outside
iptables -A INPUT -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -A INPUT -m state --state NEW -i !$OIFACE -j ACCEPT
iptables -A FORWARD -i $OIFACE -o $IIFACE -m state --state ESTABLISHED,RELATED -j ACCEPT

# Allow outgoing connections from the LAN side.
iptables -A FORWARD -i $IIFACE -o $OIFACE -j ACCEPT

# Masquerade.
iptables -t nat -A POSTROUTING -o $OIFACE -j MASQUERADE

# Don't forward from the outside to the inside.
iptables -A FORWARD -i $OIFACE -o $OIFACE -j REJECT
iptables -A FORWARD -i $IIFACE -o $OIFACE -j REJECT

echo "POST-CONNECT: setting default route"

ip route del default
route add default gw $GW dev $OIFACE

echo "POST-CONNECT: setting nameservers"

echo $DNS1 > /etc/resolv.conf
echo $DNS2 >> /etc/resolv.conf

echo "POST-CONNECT: synchronizing clock"
/sbin/ntpd -n -q -p 132.163.96.4

echo "POST-CONNECT: done"
