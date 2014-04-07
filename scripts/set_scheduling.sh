#!/bin/sh

ps_grep="_[r]os2fri"
topic_grep="lwr*/direct/set_joint"
pids=$(ps waux | egrep "$ps_grep" | awk '{ print $2 }')

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

for pid in $pids
do
	echo "$0: Using sudo to set PID $pid to SCHED_RR with highest priority"
	sudo chrt -r -p 99 $pid
done
