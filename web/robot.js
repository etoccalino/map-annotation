/**
 * HANDLE CONNECTION TO THE ROBOT.
 *
 * The module exposes a singleton object 'ROBOT'.
 *
 * Call ROBOT.init() before calling any other method. After that, you may
 * ROBOT.createGoal() and use the retuned value to ROBOT.sendGoal() and
 * ROBOT.cancelGoal().
 */

var ROBOT = {
  ip: '10.1.0.21',
  debug: true
}

/**
 * Initialize the ROBOT object and connect to the robot.
 *
 * @params an object with keys:
 *  * ip    : the IP address of the robot, as a string
 *  * debug : a boolean to signal debugging output.
 */
ROBOT.init = function (params) {
  params = params || {};
  ROBOT.debug = (params.debug !== undefined) ? params.debug : ROBOT.debug;
  ROBOT.ip = params.ip || ROBOT.ip;
  if (! ROBOT.ip) {
    throw Error('Must provide a robot IP address.');
  }

  // ROS CONNECTION
  ROBOT.ros = new ROSLIB.Ros({
    url : 'ws://' + ROBOT.ip + ':9090'
  });
  if (ROBOT.debug) {
    ROBOT.ros.on('error', function(error) {
      ROBOT._debug_log_object(error);
    });
    ROBOT.ros.on('connection', function () {
      ROBOT._debug_log_msg('ROS connection successfull.');
    });
  }

  // ACTION CLIENT CONNECTION
  ROBOT.moveBaseActionClient = new ROSLIB.ActionClient({
    ros : ROBOT.ros,
    serverName : '/move_base',
    actionName : 'move_base_msgs/MoveBaseAction'
  });
}


/**
 * Create a Goal object, suitable to pass to sendGoal() and cancelGoal(). Read
 * the params carefully, they're confusing.
 *
 * @params an object with keys:
 *  * x : x coordinate of the position
 *  * y : y coordinate of the position
 *  * z : z coordinate of the orientation
 *  * w : w coordinate of the orientation
 *
 * Sending the returned goal will send the robot to:
 *  position: x, y, 0
 *  orientation: 0, 0, z, w
 */
ROBOT.createGoal = function (x, y, z, w) {

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
          orientation: { x: 0, y: 0, z: z, w: w }
        }
      }
    }
  });

  if (ROBOT.debug) {
    moveBaseGoal.on('result', function(result) {
      ROBOT._debug_log_msg('Final result from robot:');
      ROBOT._debug_log_object(result);
    });
  }

  return moveBaseGoal;
}

/**
 * Send a goal to the robot.
 *
 * @params an object with keys:
 *  * goal : a goal object, as returned by createGoal()
 *  * callback : callback function to call on success and error condition
 *
 * The goal should be constructed with createGoal(). The callback parameter
 * should expect an error object (which will by null if no error ocurred) and a
 * result object (which will be null if an error ocurred).
 */
ROBOT.sendGoal = function (goal, callback) {
  if (callback) {
    goal.on('result', function () {
      callback(null, result);
    });
    goal.on('timeout', function () {
      callback({error: 'timeout'}, null);
    });
  }

  ROBOT._debug_log_msg('Sending goal to robot...');
  goal.send();
  ROBOT._latestGoal = goal;
  ROBOT._debug_log_msg('...goal sent to robot.');
}

/**
 * Cancel a goal sent to the robot.
 *
 * @params an object with keys:
 *  * goal : a goal object, as returned by createGoal() (optional)
 *
 * The goal should be constructed with createGoal(). If not provided, the last
 * goal passed to sendGoal() is cancelled (if any).
 */
ROBOT.cancelGoal = function (goal) {
  var g = goal || ROBOT._latestGoal;
  ROBOT._debug_log_msg('Cancelling goal...');
  g.cancel();
  ROBOT._debug_log_msg('...goal canceled.')
}

// Conditionally log a string to the console.
ROBOT._debug_log_msg = function (msg) {
  if (ROBOT.debug) {
    console.log(msg);
  }
}
// Conditionally show an object in the console.
ROBOT._debug_log_object = function (obj) {
  if (ROBOT.debug) {
    console.dir(obj);
  }
}
