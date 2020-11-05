// Connecting to ROS
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
  let pubRecalib = document.getElementById("pub_recalib")

  let lf_cw  =  document.getElementById("lf_cw ");
  let lf_acw =  document.getElementById("lf_acw");
  let lb_cw  =  document.getElementById("lb_cw ");
  let lb_acw =  document.getElementById("lb_acw");
  let rf_cw  =  document.getElementById("rf_cw ");
  let rf_acw =  document.getElementById("rf_acw");
  let rb_cw  =  document.getElementById("rb_cw ");
  let rb_acw =  document.getElementById("rb_acw");

  let battery =  document.getElementById("battery");

  let temp_health = document.getElementById("temp_health");
  let temp_value = document.getElementById("temp_value");

  let volt_health = document.getElementById("volt_health");
  let volt_value = document.getElementById("volt_value");

  let current_health = document.getElementById("current_health");
  let current_value = document.getElementById("current_value");

  var svg_lf_text = document.getElementById("svg_lf_text").textContent;
  

  var target_angle = new ROSLIB.Topic({
    ros: ros,
    name: '/target_angle',
    messageType: 'geometry_msgs/Twist'

  });

  var encoder_positions = new ROSLIB.Topic({
    ros: ros,
    name: '/encoder_positions',
    messageType: 'geometry_msgs/Twist'

  });

  var encoder_speed = new ROSLIB.Topic({
    ros: ros,
    name: '/encoder_speed',
    messageType: 'geometry_msgs/Twist'

  });

  var battery_info = new ROSLIB.Topic({
    ros: ros,
    name: '/bms_data',
    messageType: 'battery_info_pub.battery_info'
  });

  var battery_twist = new ROSLIB.Topic({
    ros: ros,
    name : "/battery_twist",
    messageType: "geometry_msgs/Twist"
  });


  var target_speed = new ROSLIB.Topic({
    ros: ros,
    name: '/target_speed',
    messageType: 'geometry_msgs/Twist'

  });

  var recalib_encoder = new ROSLIB.Topic({
    ros : ros,
    name : '/recalibrate_encoder',
    messageType : 'std_msgs/Int32'
  });

  var cnt = 0;

  

  function update_batt_data(data){
    var T = data.linear.x;
    var V = data.linear.y;
    var I = data.linear.z;

    Plotly.extendTraces("temp",{y:[[T]]}, [0]);
    Plotly.extendTraces("volt",{y:[[V]]}, [0]);
    Plotly.extendTraces("current",{y:[[I]]}, [0]);

    var T_percent = T/360;


    temp_health.value = (T/360) *100;
    temp_value.innerHTML="Temperature:"+  T.toString() +"K";

    volt_health.value = (V/360) *100;
    volt_value.innerHTML="Voltage:"+  V.toString() +"V";

    current_health.value = (I/360) *100;
    current_value.innerHTML="Current:"+  I.toString() +"A";

    


    if(cnt > 30){
      Plotly.relayout("temp", {
        xaxis: {
          range: [cnt-30,cnt]
        }
        });
      };
  
      if(cnt > 30){
        Plotly.relayout("volt", {
          xaxis: {
            range: [cnt-30,cnt]
          }
          });
        };

        if(cnt > 30){
          Plotly.relayout("current", {
            xaxis: {
              range: [cnt-30,cnt]
            }
            });
          };

          cnt++
  
  };

  function update_angle(data){
    var lf = data.linear.x;
    var rf= data.linear.y;
    var lb= data.linear.z;
    var rf = data.angular.x

    document.getElementById("svg_lf_text").textContent = lf;
    document.getElementById("svg_rf_text").textContent = rf;
    document.getElementById("svg_lb_text").textContent = lb;
    document.getElementById("svg_rb_text").textContent = rf;

    document.getElementById("svg_lf_path").style.strokeDasharray = lf;
    document.getElementById("svg_rf_path").style.strokeDasharray = rf;
    document.getElementById("svg_lb_path").style.strokeDasharray = lb;
    document.getElementById("svg_rb_path").style.strokeDasharray = rf;




  }

  battery_info.subscribe(function(msg){console.log("batt info sub")});
  battery_twist.subscribe(function(msg){update_batt_data(msg)})
  encoder_positions.subscribe(function(msg){update_angle(msg)})



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
/*
  pubRecalib.onclick  = function(){
    encoder_idx = document.getElementById("encoder_idx");
    recalib_encoder.publish(encoder_idx);
  };
  
*/

  submitAngle.onclick = function(){
    angle_lf  = parseFloat(document.getElementById("lf_rot").value);
    angle_lb  = parseFloat(document.getElementById("lb_rot").value);
    angle_rf  = parseFloat(document.getElementById("rb_rot").value);
    angle_rb  = parseFloat(document.getElementById("rf_rot").value);
    alert("sending target angle" );
    console.log(angle_lf);
    console.log(angle_lb);
    console.log(angle_rf);
    console.log(angle_rb);

    var twist_angle = new ROSLIB.Message({
        linear : {
          x : angle_lf,
          y : angle_lb,
          z : angle_rf
        },
        angular : {
          x : angle_rb,
          y : 0.0,
          z : 0.0,
        }
      });

    target_angle.publish(twist_angle);
  };


  Plotly.plot("temp",[{
    y:[0],
    type:"line"
    }]);

  Plotly.plot("volt",[{
      y:[0],
      type:"line"
      }]);
    
  Plotly.plot("current",[{
      y:[0],
      type:"line"
      }]);



  
  
    /*   
    lf_cw.onclick   = function(){
        //modbus set speed
    };
    lf_acw.onclick  = function(){
        //modbus set speed
    };
    lb_cw.onclick   = function(){
        //modbus set speed
    };
    lb_acw.onclick  = function(){
        //modbus set speed
    };
    rf_cw.onclick   = function(){
        //modbus set speed
    };
    rf_acw.onclick  = function(){
        //modbus set speed
    };
    rb_cw.onclick   = function(){
        //modbus set speed
    };
    rb_acw.onclick  = function(){
        //modbus set speed
    };

  // Subscribing to command velocity

*/

