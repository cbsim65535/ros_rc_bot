angular.module("Joy", [])
    .controller("JoyCtrl", ($scope) => {
        var interval
        var accel = 0
        var steer = 0
        var accel_scale = 1
        var steer_scale = 0.5

        ip = getParam("ip")
        if (!ip) {
            ip = "localhost"
        }
        var ros = new ROSLIB.Ros({
            url: 'ws://' + ip + ':9090'
        });
        var cmdVel
        ros.on('connection', function () {
            console.log('Connected to websocket server.');
            cmdVel = new ROSLIB.Topic({
                ros: ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            });
        });

        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error);
        });

        ros.on('close', function () {
            console.log('Connection to websocket server closed.');
        });

        $("#box").on("touchstart", (e) => {
            if (!interval) {
                interval = setInterval(function () {


                    var twist = new ROSLIB.Message({
                        linear: {
                            x: accel * accel_scale,
                            y: 0,
                            z: 0
                        },
                        angular: {
                            x: 0,
                            y: 0,
                            z: steer * steer_scale
                        }
                    });

                    cmdVel.publish(twist)

                }, 100)
            }
        })

        $("#box").on("touchmove", (e) => {
            var w = $(e.currentTarget).width()
            var h = $(e.currentTarget).height()
            var sx = (e.targetTouches[0].clientX / w - 0.5) * -2
            var sy = (e.targetTouches[0].clientY / h - 0.5) * -2
            accel = sy
            steer = sx
        })

        $("#box").on("touchend", (e) => {
            console.log("##############")
            clearInterval(interval)
            interval = null
            accel = 0
            steer = 0
            var twist = new ROSLIB.Message({
                linear: {
                    x: 0,
                    y: 0,
                    z: 0
                },
                angular: {
                    x: 0,
                    y: 0,
                    z: 0
                }
            });

            cmdVel.publish(twist)
        })

    })

function getParam(sname) {

    var params = location.search.substr(location.search.indexOf("?") + 1);

    var sval = "";

    params = params.split("&");

    for (var i = 0; i < params.length; i++) {

        temp = params[i].split("=");

        if ([temp[0]] == sname) { sval = temp[1]; }

    }

    return sval;

}