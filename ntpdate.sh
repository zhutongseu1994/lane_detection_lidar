#########################################################################
# File Name: ntpdate.sh
# Author: zhanglincan
# mail: 15062261019@163.com
# Created Time: 2020年12月02日 星期三 17时45分47秒
#########################################################################
#/bin/sh
#sleep 15
#hwclock -s
while :
do
	strA=$(/usr/sbin/ntpdate 192.168.1.62)
	strB="adjust time server"

	case $strA in 
	  *"$strB"*) 
	  echo Enemy Spot ;;
	  *) 
	  echo nope ;;
	esac

#hwclock --systohc
sleep 5
done
