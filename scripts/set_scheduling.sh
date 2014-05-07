#!/bin/sh

topic_grep="lwr.*/direct/set_joint"
echo "$0: Waiting for topics $topic_grep to become available"
while true
do
	topics_available=$(rostopic list | egrep "$topic_grep")
	if [ ! -z "$topics_available" ] ; then
		break
	else
		sleep 1
	fi
done

ps_grep="_[r]os2fri"
pids=$(ps waux | egrep "$ps_grep" | awk '{ print $2 }')
for pid in $pids
do
	echo "$0: Using sudo to set PID $pid to SCHED_RR and realtime IO class"
	sudo chrt -r -p 90 $pid
	sudo ionice -c 1 -p $pid
done

echo "Done going to sleep for a year"
sleep 31536000
