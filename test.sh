#!/bin/sh

thisString="1 2 3 4 5" # 源字符串
searchString="1 2" # 搜索字符串
case $thisString in 
  *"$searchString"*) echo Enemy Spot ;;
  *) echo nope ;;
esac
