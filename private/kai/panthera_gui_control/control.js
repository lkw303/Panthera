/ Connecting to ROS
  // -----------------

  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Publishing a Topic
  // ------------------

 

  let submitSpeed = document.getElementById("submit_speed");
  let submitAngle = document.getElementById("submit_angle");

  let lf_cw  =  document.getElementById("lf_cw ");
  let lf_acw =  document.getElementById("lf_acw");
  let lb_cw  =  document.getElementById("lb_cw ");
  let lb_acw =  document.getElementById("lb_acw");
  let rf_cw  =  document.getElementById("rf_cw ");
  let rf_acw =  document.getElementById("rf_acw");
  let rb_cw  =  document.getElementById("rb_cw ");
  let rb_acw =  document.getElementById("rb_acw");


  var target_angle = new ROSLIB.Topic({
    ros: ros,
    name: '/target_angle',
    messageType: 'geometry_msgs/Twist'

  });


  var target_speed = new ROSLIB.Topic({
    ros: ros,
    name: '/target_speed',
    messageType: 'geometry_msgs/Twist'

  });
  

  submitSpeed.onclick = function(){
    speed_lf    = document.getElementById("lf_trans").value;
    speed_lb    = document.getElementById("lb_trans").value;
    speed_rf    = document.getElementById("rb_trans").value;
    speed_rb   = document.getElementById("rf_trans").value;
    alert("sending target speed");
  
    var twist_speed = new ROSLIB.Message({
        linear : {
          x : speed_lf,
          y : speed_lb,
          z : speed_rf
        },
        angular : {
          x : speed_rb,
          y : 0,
          z : 0
        }
      });

    target_speed.publish(twist_speed);
  };


  submitAngle.onclick = function(){
    angle_lf  = document.getElementById("lf_rot").value;
    angle_lb  = document.getElementById("lb_rot").value;
    angle_rf  = document.getElementById("rb_rot").value;
    angle_rb  = document.getElementById("rf_rot").value;
    alert("sending target angle" );
    console.log(text);

    var twist_angle = new ROSLIB.Message({
        linear : {
          x : angle_lf,
          y : angle_lb,
          z : angle_rf
        },
        angular : {
          x : angle_rb,
          y : -0.2,
          z : -0.3
        }
      });

    target_angle.publish(newText);
  };

    lf_cw.onclick   = function{
        //modbus set speed
    }
    lf_acw.onclick  = function{
        //modbus set speed
    }
    lb_cw.onclick   = function{
        //modbus set speed
    }
    lb_acw.onclick  = function{
        //modbus set speed
    }
    rf_cw.onclick   = function{
        //modbus set speed
    }
    rf_acw.onclick  = function{
        //modbus set speed
    }
    rb_cw.onclick   = function{
        //modbus set speed
    }
    rb_acw.onclick  = function{
        //modbus set speed
    }

  // Subscribing to command velocity

 


   
  cmd_vel.subscribe(function(msg){print_vel(msg)});

  