#!/bin/sh

ETHIF="bridge0"
PHOST1="10.7.254.254"

case `uname -n` in
	mdm9607)
		echo "GENERIC" && exit 0
		;;
	mdm9607-01)
		echo "REFERENCE" && exit 0
		;;
	bsn-ksc)
		echo "BSN" && exit 0
		;;
	bs-ksc)
		echo "BS" && exit 0
		;;
	*)
		echo "UNKNOWN" && exit 1
		;;
esac

# check ethernet status
ETHSTS=$(ip link show ${ETHIF} | grep -o "<.*>" | tr -d "<>")
echo ${ETHSTS} | grep -o "NO-CARRIER" && exit 1

# ping host
ping -c 3 ${PHOST1} || exit 1

ping -c 3 ${PHOST2} || exit 1

exit 0 # all tests passed

