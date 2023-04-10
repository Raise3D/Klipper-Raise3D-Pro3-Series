#!/bin/bash
res=""
ip="192.168.3.114"
if [ $# -ge 1 ];then
	if [ $1 = "all" ];then
		echo "all updating"
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/klippy /home/ubuntu/klipper-master/
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/klipper.bin /home/ubuntu/klipper-master/out
                sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/compile_time_request.txt /home/ubuntu/klipper-master/out
                #sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/compile_time_request.o /home/ubuntu/klipper-master/out
                #sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/compile_time_request.d /home/ubuntu/klipper-master/out
                sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/klipper.dict /home/ubuntu/klipper-master/out
		sshpass -p '1' scp  zhaidq@$ip:/home/zhaidq/klipper-master/test/comm.py /home/ubuntu/klipper-master/test
		sshpass -p '1' scp zhaidq@$ip:/home/zhaidq/klipper-master/printer.cfg /home/ubuntu/klipper-master/
		res="ok to update all"
	elif [ $1 = "klippy" ];then
		echo "klippy updating.."
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/klippy /home/ubuntu/klipper-master
		res="ok to update klippy pro"
	elif [ $1 = "bin" ];then
		echo "bin updating.."
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/klipper.bin /home/ubuntu/klipper-master/out
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/compile_time_request.txt /home/ubuntu/klipper-master/out
		#sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/compile_time_request.o /home/ubuntu/klipper-master/out
		#sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/compile_time_request.d /home/ubuntu/klipper-master/out
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/out/klipper.dict /home/ubuntu/klipper-master/out
		res="ok to update bin and out pro"
	elif [ $1 = "comm" ];then
		echo "comm updating.."
		sshpass -p '1' scp  zhaidq@$ip:/home/zhaidq/klipper-master/test/comm.py /home/ubuntu/klipper-master/test
		res="ok to update comm.py"
	elif [ $1 = "src" ];then
		echo "mcu src updating.."
		sshpass -p '1' scp -r zhaidq@$ip:/home/zhaidq/klipper-master/src /home/ubuntu/klipper-master/
		res="mcu src update ok"
	elif [ $1 = "cfg" ];then
		echo "printer.cfg updating.."
		sshpass -p '1' scp zhaidq@$ip:/home/zhaidq/klipper-master/printer.cfg /home/ubuntu/klipper-master/
		res="ok to update printer.cfg"
	elif [ $1 = "gcode" ];then
		echo "test gcode updating.."
		sshpass -p '1' scp -r zhaidq@$ip:/mnt/hgfs/share_ubuntu/gcode/* /home/ubuntu/klipper-master/gcode/
		res="ok to update test gcode"
	else 
		res="None"
	fi
fi
if [ $? -eq 0 -a "$res" != "" ];then
	echo "$res"
else
	echo "fail to update $1"
fi
