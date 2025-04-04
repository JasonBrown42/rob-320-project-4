# cd solution
# chmod +x bin/rixhub
./bin/rixhub &
rixhub_pid=$!
sleep 1

./build/rsp robots/simple_bot.json &
rsp_pid=$!
sleep 1

./build/jsp robots/simple_bot.json base_to_upper_arm --rate 1 &
jsp_pid=$!
sleep 1

./build/tf_recorder test/test_tf_msgs.bin &
tf_recorder_pid=$!
sleep 1

./build/joint_publisher base_to_upper_arm --rate 1
sleep 1

kill -INT $rixhub_pid
sleep 1
kill -INT $rsp_pid
sleep 1
kill -INT $jsp_pid
sleep 1
kill -INT $tf_recorder_pid
sleep 1

./build/compare_rsp_test