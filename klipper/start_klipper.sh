#!/bin/bash
if [ $# -eq 0 ];then
	echo "load config file printer.cfg"
	sudo python klippy/klippy.py printer.cfg -l /opt/Raise3D/logs/system/klippy.log
elif [ $# -ge 1 ];then
	echo "load config file $1"
	ls $1 &>/dev/null
	if [ $? -eq 0 ]
	then
		if echo "$1" | grep -q -E '\.cfg$'
        	then
			echo "load config file $1"
	        	sudo python klippy/klippy.py $1 -l /opt/Raise3D/logs/system/klippy.log
		else
                	echo "ivalid config file"
        	fi
	else
		echo "config file not exsit"
	fi
fi
