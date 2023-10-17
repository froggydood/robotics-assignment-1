num_runs_per_bag=3
run_length_secs=60
num_steps=6
step=1
start=8

initial_pose_1="header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.17085
      y: 0.15702
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.00487
      w: 1
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 99999.0]"

initial_pose_2="header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 10.0626
      y: -4.17568
      z: 0.0
    orientation:
      x: 0.0
      y: 0.57908
      z: 0.81527
      w: 1
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 99999.0]"

rm -rf ./measure-results.txt

for current_step in $(seq 0 $(($num_steps-1)))
do
  current=$(echo "$start+$current_step*$step"| bc)
  sed "s@%VALUE%@$current@" node.launch > node-test-$current_step.launch
  chmod 777 ./node-test-$current_step.launch
  echo "Value $current, Step Num $current_step" >> ./measure-results.txt
  echo "-----------------" >> ./measure-results.txt
  for run in $(seq 1 $num_runs_per_bag)
  do
    for pose_index in $(seq 1 2)
    do
      initial_pose=$initial_pose_1
      if [[ "$pose_index" == "2" ]];
      then
        initial_pose=$initial_pose_2
      fi
      timeout $(($run_length_secs+10)) roslaunch ./node-test-$current_step.launch &
      sleep 10
      timeout $run_length_secs rosbag play -r 0.5 --clock ~/catkin_ws/src/pf_localisation/data/sim_data/simpath1.bag &
      echo "Setting initial pose $pose_index"
      rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "$initial_pose"
      sleep $run_length_secs
      sleep 3
      echo "Run $run intial pose $pose_index" >> ./measure-results.txt
      cat ./results.txt >> ./measure-results.txt
      echo "" >> ./measure-results.txt
    done
  done
  rm -rf node-test-$current_step.launch
done
