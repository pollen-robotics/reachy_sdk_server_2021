# Run "ros2 launch reachy_kinematics kinematics.launch.py"

# source ROS2 Foxy setup file.
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
source $HOME/reachy_ws/install/setup.bash

# Start the ROS2 launch file
model=$(reachy-identify-model)
zuuu_model=$(reachy-identify-zuuu-model)
if [ $zuuu_model = none ];
then
    if [[ $model = full_kit* ]] || [[ $model = starter_kit* ]] || [[ $model = mini ]];
    then
        ros2 launch reachy_sdk_server run_everything.launch.py
    else
        ros2 launch reachy_sdk_server run_everything_no_head.launch.py
    fi
else 
    if [[ $model = full_kit* ]] || [[ $model = starter_kit* ]] || [[ $model = mini ]];
    then
        ros2 launch reachy_sdk_server run_everything_and_zuuu.launch.py
    else
        ros2 launch reachy_sdk_server run_everything_no_head_and_zuuu.launch.py
    fi
fi

# if [ $zuuu_model != none ];
# then
#     echo "starting mobile base launch file"
#     ros2 launch zuuu_description zuuu_bringup_low_level_only.launch.py
# else
#     echo "Model with no mobile base"
# fi
