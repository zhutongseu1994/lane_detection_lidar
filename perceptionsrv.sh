#/bin/sh

cd ~/rslidar_perception_ws
while :
do
topic_state=$(rostopic list | grep lslidar_point_cloud | wc -l)
segment_running=$(ps -ef | grep "segment_node" | grep -v "grep")
cluster_running=$(ps -ef | grep "cluster_node" | grep -v "grep")
track_running=$(ps -ef | grep "track_node" | grep -v "grep")
if [ $topic_state -eq 1 ];then
	if [ "$segment_running" ];then
		echo "segment_node is running!"
	else
		nohup ./devel/lib/segment/segment_node > /dev/null 2>&1 &
	fi
	if [ "$cluster_running" ];then
		echo "cluster_node is running!"
	else
		nohup ./devel/lib/cluster/cluster_node > /dev/null 2>&1 &
	fi

	if [ "$track_running" ];then
		echo "track_node is running!"
	else
		nohup ./devel/lib/track/track_node > /dev/null 2>&1 &
	fi
else
	if [ "$segment_running" ];then
		kill -9 `ps -ef | grep "segment_node" | grep -v grep | awk '{print $2}'`
	fi
	if [ "$cluster_running" ];then
		kill -9 `ps -ef | grep "cluster_node" | grep -v grep | awk '{print $2}'`
	fi
	if [ "$track_running" ];then
		kill -9 `ps -ef | grep "track_node" | grep -v grep | awk '{print $2}'`
	fi
fi
sleep 1
find ./log -mtime +7 -type f -name "*.log" -exec rm -rf {} \;

done
