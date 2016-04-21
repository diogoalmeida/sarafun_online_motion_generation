#!/bin/bash
sleep 5
rostopic pub -1 /hqp_node/ref_pose_topic geometry_msgs/PoseStamped '{ header: { frame_id: "" }, pose: { position: {x: 0.0,y: 0.3,z: 0.2 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

rosservice call /hqp_node/addObs_HQP ['{name: ob1,  pose: {position: {x: 0.2,y: 0.3,z: 0.1 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.07,y: 0.07,z: 0.07 }}']
rosservice call /hqp_node/addObs_HQP ['{name: ob2,  pose: {position: {x: 0.25,y: 0.25,z: 0.15 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.05,y: 0.05,z: 0.05 }}']
sleep 2
#rostopic pub /hqp_node/ref_pose_topic geometry_msgs/PoseStamped '{ header: { frame_id: "" }, pose: { position: {x: 0.0,y: 0.3,z: 0.2 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

rostopic pub -1 /hqp_node/ref_pose_topic geometry_msgs/PoseStamped '{ header: { frame_id: "" }, pose: { position: {x: 0.5,y: 0.2,z: 0.1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

sleep 1

rosservice call /hqp_node/addObs_HQP ['{name: ob2,  pose: {position: {x: 0.35,y: 0.25,z: 0.1 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.05,y: 0.05,z: 0.05 }}']

sleep 6
rosservice call /hqp_node/addObs_HQP ['{name: ob2,  pose: {position: {x: 0.25,y: 0.25,z: 0.15 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.05,y: 0.05,z: 0.05 }}']
#rosservice call /hqp_node/addObs_HQP ['{name: ob2,  pose: {position: {x: 0.35,y: 0.25,z: 0.1 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.01,y: 0.01,z: 0.01 }}']


sleep 1
rosservice call /hqp_node/addObs_HQP ['{name: ob1,  pose: {position: {x: -0.2,y: 0.3,z: 0.1 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.07,y: 0.07,z: 0.07 }}']
rosservice call /hqp_node/addObs_HQP ['{name: ob2,  pose: {position: {x: -0.35,y: 0.25,z: 0.1 }, orientation: {x: 0., y: 0.,z: 0., w: 0.}}, axes: {x: 0.05,y: 0.05,z: 0.05 }}']
