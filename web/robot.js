var ROBOT = {
  ip: '10.1.0.21',
}

// ROS CONNECTION

ROBOT.ros = new ROSLIB.Ros({
  url : 'ws://' + ROBOT.ip + ':9090'
});
ROBOT.ros.on('error', function(error) {
  console.log(error);
});
ROBOT.ros.on('connection', function () {
  console.log('ROS connection successfull.');
});


// ACTION CLIENT CONNECTION

ROBOT.moveBaseActionClient = new ROSLIB.ActionClient({
  ros : ROBOT.ros,
  serverName : '/move_base',
  actionName : 'move_base_msgs/MoveBaseAction'
});


// Create a goal.

ROBOT.createGoal = function (x, y, w) {

  var moveBaseGoal = new ROSLIB.Goal({
    actionClient : ROBOT.moveBaseActionClient,
    goalMessage : {
      target_pose: {
        header: {
          seq: 312791, // gibberish
          stamp: {
            secs: 1376659578, // gibberish
            nsecs: 712564133  // gibberish
          },
          frame_id: '/map'
        },
        pose: {
          position: { x: x, y: y, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: w }
        }
      }
    }
  });

  moveBaseGoal.on('result', function(result) {
    console.log('Final result from robot:');
    console.dir(result);
  });

  return moveBaseGoal;
}

ROBOT.sendGoal = function (goal) {
  console.log('Sending goal to robot...');
  goal.send();
  ROBOT._latestGoal = goal;
  console.log('...goal sent to robot.');
}

ROBOT.cancelGoal = function (goal) {
  var goal = goal || ROBOT._latestGoal;
  console.log('Cancelling goal...');
  goal.cancel();
  console.log('...goal canceled.')
}
