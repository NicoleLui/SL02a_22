//import 'dart:html';

import 'package:flutter/material.dart';
import 'package:control_pad/control_pad.dart'; //no null safety

import 'package:roslib/roslib.dart';
import 'dart:math';
import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';
import 'package:ros_nodes/ros_nodes.dart';
import 'package:ros_nodes/messages/std_msgs/String.dart';

import 'package:flutter/services.dart';
import 'dart:ui' as ui;
import 'dart:ui';
import 'dart:io';
//import 'dart:core';
//\x03
//subscribe to get map: /map_merge/map
// type:nav_msgs/OccupancyGrid
void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const MyHomePage(title: 'FYP testing'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

const kCanvasSize = 150.0;
ByteData? imgBytes;

class _MyHomePageState extends State<MyHomePage> with SingleTickerProviderStateMixin{
  late Ros ros;
  late Topic chatter;
  late Topic counter;
  late Topic cmd_vel;
  late Topic imu;
  late Topic camera;
  late Topic scan;
  late Topic mobile_input;
  late Topic mobile_drawing;
  late Topic mobile_mode;
  int _index = 0;
  late final TabController _controller;
  late final Timer _timer;

  Color selectedColor = Colors.indigoAccent;
  double strokeWidth = 4.0;
  List<DrawingPoints> points = [];
  bool showBottomList = false;
  double opacity = 1.0;

  void _move(double _degrees, double _distance) {
    print(
        'Degree:' + _degrees.toString() + ' Distance:' + _distance.toString());
    double radians = _degrees * ((22 / 7) / 180);
    double linear_speed = cos(radians) * _distance;
    double angular_speed = -sin(radians) * _distance;

    publishCmd(linear_speed, angular_speed);
  }

  void _circulate() {
    (_index != 1) ? _index++ : _index = 0;
    _controller.animateTo(_index);
    setState(() {});
  }

  @override
  void initState() {
    super.initState();
    _controller = TabController(
      length: 2,
      initialIndex: _index,
      vsync: this,
    );
    _timer = Timer.periodic(
      const Duration(seconds: 1),
          (_) => _circulate(),
    );


    ros = Ros(url: 'ws://192.168.1.102:11311'); //101 104 102 11311
    //var ros2 = RosNode('http://192.168.43.145:11311/', '192.168.43.92', 51235);
    chatter = Topic(
        ros: ros,
        name: '/chatter',
        type: "std_msgs/String",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);

    cmd_vel = Topic(
        ros: ros,
        name: '/cmd_vel',
        type: "geometry_msgs/Twist",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);

    counter = Topic(
      ros: ros,
      name: '/counter',
      type: "std_msgs/String",
      reconnectOnClose: true,
      queueSize: 10,
      queueLength: 10,
    );

    imu = Topic(
      ros: ros,
      name: '/imu',
      type: 'sensor_msgs/Imu',
      queueSize: 10,
      queueLength: 10,
    );

    camera = Topic(
      ros: ros,
      name: '/usb_cam/image_raw', // camera/image/compressed'
      type: 'sensor_msgs/CompressedImage',
      queueSize: 30,
      queueLength: 30,
    );

    scan = Topic(
      ros: ros,
      name: '/scan',
      type: 'sensor_msgs/CompressedImage',
      queueSize: 10,
      queueLength: 10,
    );
    //super.initState();
    mobile_input = Topic(
      ros: ros,
      name: 'mobile_input',
      type: 'std_msgs/String',
    );

    mobile_drawing = Topic(
      ros: ros,
      name: 'mobile_drawing',
      type: 'std_msgs/CompressedImage',
      queueSize: 30,
      queueLength: 30,
    );

    mobile_mode = Topic(
      ros: ros,
      name: 'mobile_mode',
      type: 'std_msgs/String',
    );
  }
//mobile_input
  @override
  void dispose() {
    _controller.dispose();
    _timer.cancel();
    super.dispose();
  }

  void initConnection() async {
    ros.connect();
    await chatter.subscribe();
    await cmd_vel.subscribe();
    await imu.subscribe();
    await camera.subscribe();
    await counter.advertise();
    await cmd_vel.advertise();
    await mobile_input.advertise();
    setState(() {});
  }
  /*void initConnection()  {
    ros.connect();
     chatter.subscribe();
     cmd_vel.subscribe();
     imu.subscribe();
     camera.subscribe();
     counter.advertise();
     cmd_vel.advertise();
     mobile_input.advertise();
     mobile_input.publish("message");
    setState(() {});
  }*/
//pub_test: node
  void publishCounter() async {
    var msg = {'data': 'hello'};
    await counter.publish(msg);
    print('done published');
  }
  void publishMobileInput(String data) async {
    var config = RosConfig(
      'turtlebot_controller', //to?
      'http://192.168.1.102:11311/',
      '192.168.1.100', //client(phone) ip
      51235, //11311 51235
    );
    try{
      print('connect');
      var client = RosClient(config);
      print('client');
      var msg = StdMsgsString();
      msg.data = data;
      print('msg');
      var topic = RosTopic('mobile_input', msg);
      print('topic');
      var publisher = await client.register(topic,
          publishInterval: Duration(milliseconds: 1000));
      print('publisher');
      publisher.startPublishing();
      publisher.publishData();
      /*var i = 0;
      Timer.periodic(
        Duration(milliseconds: 500),
            (_) {
          i += 1;
          topic.msg.data = i.toString();
        },
      );*/
      //publisher.stopPublishing();
      print('done');
    } catch (e) {
      print(e);
    }
    /*var client = RosClient(config);
    var topic = RosTopic('mobile_input', StdMsgsString());
    await client.unsubscribe(topic);
    var msg = {data};
    //mobile_input.publish(msg);
    var publisher = RosPublisher(topic, msg.toString());
    // publisher.register();
    //var publisher = RosPublisher(topic, 11311, publishInterval: Duration(milliseconds: 100));
    print('done published');*/

  }

  void publishCmd(double _linear_speed, double _angular_speed) async {
    var linear = {'x': _linear_speed, 'y': 0.0, 'z': 0.0};
    var angular = {'x': 0.0, 'y': 0.0, 'z': _angular_speed};
    var twist = {'linear': linear, 'angular': angular};
    await cmd_vel.publish(twist);
    print('cmd published');
    publishCounter();
  }

  void destroyConnection() async {
    await chatter.unsubscribe();
    await cmd_vel.unsubscribe();
    await imu.unsubscribe();
    await camera.unsubscribe();
    await counter.unadvertise();
    await mobile_input.unadvertise();
    await ros.close();
    setState(() {});
  }

  void publishmsg(String msg) async{
    var config = RosConfig(
      'pub_test', //to?
      'http://192.168.1.102:11311/',
      '192.168.1.102',
      11311,
    );
    var client = RosClient(config);
    var topic = RosTopic('mobile_input', StdMsgsString());
    await client.unregister(topic);

    /*var publisher = await client.register(topic,
        publishInterval: Duration(milliseconds: 1000));

    var publisher = RosPublisher(topic, 'mobile_input', msg, config,
        publishInterval: Duration(milliseconds: 100));
    publisher.register();*/

    var publisher = RosPublisher(topic, msg);

    var i = 0;
    Timer.periodic(
      Duration(milliseconds: 500),
          (_) {
        i += 1;
        topic.msg.data = i.toString();
      },
    );
    
  }

  bool isManual = false;
  bool isGraphical = true;
  void toImage() async{
    //publish imgBytes
    final file = File('C:/Users/Nicole Lui/Desktop/HKUST/FYP/image_output/output.png');
    await file.writeAsBytes(imgBytes!.buffer.asInt8List());
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: StreamBuilder<Object>(
        stream: ros.statusStream,
        builder: (context, snapshot){
          return Center(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.center,
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Container(
                  child:isGraphical?
                  /*Container(
                    width: 100,
                    height: 100,
                    color: Colors.yellow,
                  )*/
                  Row(mainAxisAlignment: MainAxisAlignment.center,
                      children:[
                        Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [GestureDetector(
                            onPanUpdate: (details) {
                              setState(() {
                                RenderBox renderBox = context.findRenderObject() as RenderBox;
                                points.add(DrawingPoints(
                                    points: renderBox.globalToLocal(details.globalPosition),
                                    paint: Paint()
                                      ..isAntiAlias = true
                                      ..color = selectedColor.withOpacity(opacity)
                                      ..strokeWidth = strokeWidth));
                              });
                            },
                            onPanStart: (details) {
                              setState(() {
                                RenderBox renderBox = context.findRenderObject() as RenderBox;
                                points.add(DrawingPoints(
                                    points: renderBox.globalToLocal(details.globalPosition),
                                    paint: Paint()
                                      ..isAntiAlias = true
                                      ..color = selectedColor.withOpacity(opacity)
                                      ..strokeWidth = strokeWidth));
                              });
                            },
                            child:Column(
                              children:[
                                Text('Draw here:)'),
                                Container(
                                color: Colors.yellow,
                                //alignment: const Alignment(0,0),
                                //mainAxisAlignment: MainAxisAlignment.center,
                                child: CustomPaint(
                                  size: Size.square(kCanvasSize), //200
                                  painter: DrawingPainter(
                                    pointsList: points,
                                  ),
                                ),
                              ),]
                            ),
                          ),]
                        ),
                        Column(
                          children: [
                            Text('Result here...'),
                            imgBytes != null
                                ? Container(
                                color: Colors.lightBlueAccent,
                                child: Image.memory(
                                  Uint8List.view(imgBytes!.buffer),
                                  width: kCanvasSize,
                                  height: kCanvasSize,
                                ))
                                : Container(color: Colors.lightBlueAccent,),
                          ],
                        ),
                        Column(
                            children: <Widget>[
                              IconButton(
                                icon: Icon(Icons.clear),
                                onPressed: () {
                                  setState(() {
                                    showBottomList = false;
                                    imgBytes = null;
                                    points.clear();
                                  });
                                }),
                              IconButton(
                                icon: Icon(Icons.check),
                                onPressed: toImage,
                              ),
                            ]
                        )
                      ])
                  :isManual?
                  StreamBuilder(
                      stream: camera.subscription,
                      builder: (context2,AsyncSnapshot<dynamic> snapshot2){
                        if (snapshot2.hasData){
                          return getImagenBase64(snapshot2.data['msg']['data']);
                        }
                        else{
                          return CircularProgressIndicator();
                        }
                      }
                  ):
                  Container(
                      width: 100,
                      height: 100,
                      color: Colors.green),
                ),
                /*Stack( //if no stack, incorrect parentdatawidget;
                  // if have stack, renderbox was not laid not and then no show
                  children: <Widget>[
                    Positioned( // dot
                      bottom: 20,
                      child: TabPageSelector(
                        controller: _controller,
                        color: Colors.black,
                        selectedColor: Colors.grey[600],
                      ),
                    ),
                 ],
                ),*/
                Row(
                  children: <Widget>[
                    Expanded(
                      child: SizedBox(
                        height: 200,
                        child: ListView( //joystick and buttons
                          //shrinkWrap: true,
                          scrollDirection: Axis.horizontal,
                          children: [
                            Container(
                                /*child: JoystickView(
                                  onDirectionChanged: (double degrees, double distance){
                                    _move(degrees, distance);
                                  },
                                )*/
                              child: Column( //bottom left
                                  mainAxisAlignment: MainAxisAlignment.center,
                                  //crossAxisAlignment: CrossAxisAlignment.stretch,
                                  children:<Widget>[
                                    IconButton(
                                      icon: Image.asset('assets/images/arrow_up.png'),
                                      iconSize: 50,
                                      onPressed: () {
                                        //mobile_input.publish("w");
                                        publishMobileInput("w");
                                        },
                                      alignment: Alignment.center,
                                    ),
                                    Row(
                                        children: <Widget>[
                                          IconButton(
                                            icon: Image.asset('assets/images/arrow_left.png'),
                                            iconSize: 50,
                                            onPressed: () {mobile_input.publish("a");},
                                          ),
                                          IconButton(
                                            icon: Image.asset('assets/images/arrow_circle.png'),
                                            iconSize: 32,
                                            onPressed: () {mobile_input.publish("s");},
                                          ),
                                          IconButton(
                                            icon: Image.asset('assets/images/arrow_right.png'),
                                            iconSize: 50,
                                            onPressed: () {mobile_input.publish("d");},
                                          ),
                                        ]
                                    ),
                                    IconButton(
                                      icon: Image.asset('assets/images/arrow_down.png'),
                                      iconSize: 50,
                                      onPressed: () {mobile_input.publish("x");},
                                      alignment: Alignment.center,
                                    ),
                                  ]
                              ),
                            ),
                            /*Container(
                                child: Column(
                                  mainAxisAlignment: MainAxisAlignment.center,
                                  children: <Widget>[
                                    TextButton(
                                        style: TextButton.styleFrom(
                                          textStyle: const TextStyle(fontSize: 40),
                                        ),
                                        onPressed: () {}, //send signal to system
                                        child: const Text('Faster')
                                    ),
                                    TextButton(
                                        style: TextButton.styleFrom(
                                          textStyle: const TextStyle(fontSize: 40),
                                        ),
                                        onPressed: () {}, //send signal to system
                                        child: const Text('Normal')
                                    ),
                                    TextButton(
                                        style: TextButton.styleFrom(
                                          textStyle: const TextStyle(fontSize: 40),
                                        ),
                                        onPressed: () {}, //send signal to system
                                        child: const Text('Slower')
                                    ),
                                  ],
                                )
                            ),*/
                            Container(
                                child: Column(
                                  mainAxisAlignment: MainAxisAlignment.center,
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: <Widget>[
                                    const Text("Mode...",
                                        style: TextStyle(fontSize: 40)),
                                    TextButton(
                                        style: TextButton.styleFrom(
                                          textStyle: const TextStyle(fontSize: 40),
                                          alignment: Alignment.centerLeft,
                                        ),
                                        onPressed: () {
                                          isManual = false;
                                          isGraphical = true;
                                          mobile_mode.publish('shape');
                                        }, //change page of camera to drawing
                                        child: const Text('Graphical')
                                    ),
                                    TextButton(
                                        style: TextButton.styleFrom(
                                          textStyle: const TextStyle(fontSize: 40),
                                          alignment: Alignment.centerLeft,
                                        ),
                                        onPressed: () {
                                          isManual = true;
                                        isGraphical = false;
                                          mobile_mode.publish('manual');
                                        }, //default, camera page
                                        child: const Text('Manual')
                                    ),
                                  ],
                                )
                            ),
                          ],
                        ),
                      )
                    ),
                  ],
                ),

                //then dots
                ActionChip(
                  label: Text(snapshot.data == Status.CONNECTED ? 'DISCONNECT': 'CONNECT'),
                  backgroundColor: snapshot.data == Status.CONNECTED ? Colors.green[300] : Colors.grey[300],
                  onPressed: (){
                    print(snapshot.data);
                    if (snapshot.data != Status.CONNECTED) {
                      initConnection();
                    } else {
                      this.destroyConnection();
                    }
                  },
                )
              ],
            )
          );
        }
      ),
    );
  }
}

Widget getImagenBase64(String imagen) {
  var _imageBase64 = imagen;
  const Base64Codec base64 = Base64Codec();
  if (_imageBase64 == null) return new Container(child: Text('null image'),);
  var bytes = base64.decode(_imageBase64);
  return Image.memory(
    bytes,
    gaplessPlayback: true,
    width: 400,
    fit: BoxFit.fitWidth,
  );
}

class DrawingPainter extends CustomPainter {
  DrawingPainter({required this.pointsList});
  List<DrawingPoints> pointsList;
  List<Offset> offsetPoints = [];

  @override
  void paint(Canvas canvas, Size size) async{
    final recorder = ui.PictureRecorder(); //!
    final canvas = Canvas(recorder,
        Rect.fromPoints(Offset(0.0, -90.0), Offset(kCanvasSize, kCanvasSize)));

    for (int i = 0; i < pointsList.length - 1; i++) {
      if (pointsList[i] != null && pointsList[i + 1] != null) {
        canvas.drawLine(pointsList[i].points+Offset(0,-90), pointsList[i + 1].points+Offset(0,-90),
            pointsList[i].paint);
      } else if (pointsList[i] != null && pointsList[i + 1] == null) {
        offsetPoints.clear();
        offsetPoints.add(pointsList[i].points);
        offsetPoints.add(Offset(
            pointsList[i].points.dx + 0.1, pointsList[i].points.dy + 0.1));
        canvas.drawPoints(PointMode.points, offsetPoints, pointsList[i].paint);
      }
    }

    final picture = recorder.endRecording(); //!
    final img = await picture.toImage(200, 200);
    final pngBytes = await img.toByteData(format: ImageByteFormat.png);
    //setState(() {
    imgBytes = pngBytes!; //!?

    // final file = File('C:\Users\Nicole Lui\Desktop\HKUST\FYP\image_output\output.png');
    // await file.writeAsBytes(pngBytes.buffer.asInt8List());
    //});
  }

  @override
  bool shouldRepaint(DrawingPainter oldDelegate) => true;
}

class DrawingPoints {
  Paint paint;
  Offset points;
  DrawingPoints({required this.points, required this.paint});
}
