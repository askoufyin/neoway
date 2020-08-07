#!/bin/sh
#
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
iptables -A INPUT -m state --state NEW -i ! ppp0 -j ACCEPT
iptables -A FORWARD -i ppp0 -o bridge0 -m state --state ESTABLISHED,RELATED -j ACCEPT

# Allow outgoing connections from the LAN side.
iptables -A FORWARD -i bridge0 -o ppp0 -j ACCEPT

# Masquerade.
iptables -t nat -A POSTROUTING -o ppp0 -j MASQUERADE

# Don't forward from the outside to the inside.
iptables -A FORWARD -i ppp0 -o ppp0 -j REJECT
iptables -A FORWARD -i eth0 -o ppp0 -j REJECT
