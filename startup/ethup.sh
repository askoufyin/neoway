#!/bin/sh
KVERSION="3.18.48"

#insmod /usr/lib/modules/${KVERSION}/kernel/drivers/net/phy/at803x.ko

echo "1" > /dev/wifi_power_en

insmod /usr/lib/modules/${KVERSION}/kernel/drivers/net/ethernet/qualcomm/emac/qcom_emac.ko

ifconfig bridge0 inet 10.7.254.23 netmask 255.255.0.0
