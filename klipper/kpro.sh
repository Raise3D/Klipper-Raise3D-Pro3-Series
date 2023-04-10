#!/bin/bash

echo "programming klipper"
#ls /dev/serial/by-id > /tmp/tmp_dev_dir

function flash_firmware()
{
	dev_root="/dev/serial/by-id/"
	dir_name=$(ls $dev_root)
	if [ $? -eq 0 ];then	
		dev_dir="$dev_root$dir_name"
		#echo "find dev = $dev_dir try to programming"
		python /opt/Raise3D/klipper-master/scripts/flash_usb.py -t sam4 -d $dev_dir /opt/Raise3D/klipper-master/out/klipper.bin &> /dev/null
		if [ $? -eq 0 ];then
			echo "pro ok"
			return 
		else 
			echo "pro not ok"
			return
		fi
	else
		echo "can't find dev"
		return "2"
	fi
}

for i in {1..3}; do
	rt=$(flash_firmware)
	echo "flash res = $rt"
	#echo "--->$rt"
	if [[ $rt = "pro ok" ]];then
		echo "flash ok ..."
		exit 0
	else
		echo "fail to flash try again $i"
		sleep 2
	fi
done
echo "fail to flash moveboard just exit"
exit 1


