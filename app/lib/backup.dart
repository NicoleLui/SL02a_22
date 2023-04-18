import 'package:flutter/material.dart';
import 'package:control_pad/control_pad.dart'; //no null safety
import 'package:roslib/roslib.dart';
import 'dart:math';
import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        // This is the theme of your application.
        //
        // Try running your application with "flutter run". You'll see the
        // application has a blue toolbar. Then, without quitting the app, try
        // changing the primarySwatch below to Colors.green and then invoke
        // "hot reload" (press "r" in the console where you ran "flutter run",
        // or simply save your changes to "hot reload" in a Flutter IDE).
        // Notice that the counter didn't reset back to zero; the application
        // is not restarted.
        primarySwatch: Colors.blue,
      ),
      home: const MyHomePage(title: 'FYP testing'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});
  // This widget is the home page of your application. It is stateful, meaning
  // that it has a State object (defined below) that contains fields that affect
  // how it looks.

  // This class is the configuration for the state. It holds the values (in this
  // case the title) provided by the parent (in this case the App widget) and
  // used by the build method of the State. Fields in a Widget subclass are
  // always marked "final".

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
  // State<MyHomePage> createState() => _MyPageTwoState();
}


class _MyHomePageState extends State<MyHomePage> with SingleTickerProviderStateMixin{
  late Ros ros;
  late Topic chatter;
  late Topic counter;
  late Topic cmd_vel;
  late Topic imu;
  late Topic camera;
  late Topic scan;
  int _index = 0;
  late final TabController _controller;
  late final Timer _timer;

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

    ros = Ros(url: 'ws://0.0.0.0:9090');
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
  }

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
    setState(() {});
  }

  void publishCounter() async {
    var msg = {'data': 'hello'};
    await counter.publish(msg);
    print('done published');
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
    await ros.close();
    setState(() {});
  }
  // void _incrementCounter() {
  //   setState(() {
  //     // This call to setState tells the Flutter framework that something has
  //     // changed in this State, which causes it to rerun the build method below
  //     // so that the display can reflect the updated values. If we changed
  //     // _counter without calling setState(), then the build method would not be
  //     // called again, and so nothing would appear to happen.
  //     _counter++;
  //   });
  // }

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
                ),
                Stack( //if no stack, incorrect parentdatawidget;
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
                ),
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
                                child: JoystickView(
                                  onDirectionChanged: (double degrees, double distance){
                                    _move(degrees, distance);
                                  },
                                )
                            ),
                            Container(
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
                            ),
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
                                        onPressed: () {}, //change page of camera to drawing
                                        child: const Text('Graphical')
                                    ),
                                    TextButton(
                                        style: TextButton.styleFrom(
                                          textStyle: const TextStyle(fontSize: 40),
                                          alignment: Alignment.centerLeft,
                                        ),
                                        onPressed: () {}, //default, camera page
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
                      this.initConnection();
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

class _MyPageTwoState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Expanded(
              child: Row( //up
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                mainAxisSize: MainAxisSize.min,
                children: [
                  const Text('Camera'),
                  Expanded(
                      child: Column(
                          children:<Widget>[
                            IconButton(
                              icon: Image.asset('assets/images/turn.png'),
                              iconSize: 58,
                              onPressed: () {},
                              alignment: Alignment.center,
                            ),
                          ]
                      )
                  )
                ],
              ),
            ),
            Flexible(
              fit: FlexFit.tight,
              child: Column( //bottom
                  mainAxisAlignment: MainAxisAlignment.start,
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: <Widget>[
                    const Text("Mode...",
                      style: TextStyle(fontSize: 40),
                    ),
                    TextButton(
                        style: TextButton.styleFrom(
                          textStyle: const TextStyle(fontSize: 40),
                          alignment: Alignment.centerLeft,
                        ),
                        onPressed: () {},
                        child: const Text('Graphical')

                    ),
                    TextButton(
                        style: TextButton.styleFrom(
                          textStyle: const TextStyle(fontSize: 40),
                          alignment: Alignment.centerLeft,
                        ),
                        onPressed: () {},
                        child: const Text('Manual')
                    ),
                  ]
              ),

            ),
          ],
        ),
      ),
    );
  }
}
