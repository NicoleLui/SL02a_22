import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:math';
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'dart:ui';
import 'dart:io';
//import 'dart:ui';

/*void main() {
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
}

class _MyHomePageState extends State<MyHomePage>{
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(

        ),
      ),
    );
  }
}
*/

//--------------------------------------------------------
/*class GridPainter extends CustomPainter {
  final double boxSize = 50;

  @override
  void paint(Canvas canvas, Size size) {
    final vLines = (size.width ~/ boxSize) + 1;
    final hLines = (size.height ~/ boxSize) + 1;

    final paint = Paint()
      ..strokeWidth = 1
      ..color = Colors.red
      ..style = PaintingStyle.stroke;

    final path = Path();

    // Draw vertical lines
    for (var i = 0; i < vLines; ++i) {
      final x = boxSize * i;
      path.moveTo(x, 0);
      path.relativeLineTo(0, size.height);
    }

    // Draw horizontal lines
    for (var i = 0; i < hLines; ++i) {
      final y = boxSize * i;
      path.moveTo(0, y);
      path.relativeLineTo(size.width, 0);
    }
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class GridWidget extends StatelessWidget {
  final CustomPainter foreground;

  const GridWidget(this.foreground, {Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(5),
      child: SafeArea(
        child: CustomPaint(
          foregroundPainter: foreground,
          painter: GridPainter(),
        ),
      ),
    );
  }
}

class PolygonPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 5
      ..color = Colors.indigoAccent
      ..style = PaintingStyle.stroke;

    // Triangle
    final triangle = Path()
      ..moveTo(150, 0)
      ..relativeLineTo(100, 100)
      ..relativeLineTo(-150, 0)
      ..close();
    canvas.drawPath(triangle, paint);

    // Square
    final square1 = Path();
    square1.moveTo(50, 150);
    square1.relativeLineTo(100, 0);
    square1.relativeLineTo(0, 100);
    square1.relativeLineTo(-100, 0);
    square1.close();
    canvas.drawPath(square1, paint);

    const square2 = Rect.fromLTWH(200, 150, 100, 100);
    canvas.drawRect(square2, paint);

    // Hexagon
    final hexagon = Path()
      ..moveTo(175, 300)
      ..relativeLineTo(75, 50)
      ..relativeLineTo(0, 75)
      ..relativeLineTo(-75, 50)
      ..relativeLineTo(-75, -50)
      ..relativeLineTo(0, -75)
      ..close();
    canvas.drawPath(hexagon, paint);

    // Cross
    final cross = Path()
      ..moveTo(150, 500)
      ..relativeLineTo(50, 0)
      ..relativeLineTo(0, 50)
      ..relativeLineTo(50, 0)
      ..relativeLineTo(0, 50)
      ..relativeLineTo(-50, 0)
      ..relativeLineTo(0, 50)
      ..relativeLineTo(-50, 0)
      ..relativeLineTo(0, -50)
      ..relativeLineTo(-50, 0)
      ..relativeLineTo(0, -50)
      ..relativeLineTo(50, 0)
      ..close();
    canvas.drawPath(cross, paint);

    // Circle
    const circleRadius = 75.0;
    const circleCenter = Offset(200, 150);
    canvas.drawCircle(circleCenter, circleRadius, paint);

    // Oval
    const ovalCenter = Offset(200, 275);
    final oval = Rect.fromCenter(center: ovalCenter, width: 250, height: 100);
    canvas.drawOval(oval, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

class CommonScaffold extends StatelessWidget {
  final String title;
  final Widget child;

  const CommonScaffold({required this.title, required this.child, Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        backgroundColor: Colors.white,
        foregroundColor: Colors.black,
        title: Text(title),
      ),
      body: SizedBox.expand(child: child),
    );
  }
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
    return MaterialApp(
      title: 'Canvas Basics',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: HomeWidget(),
    );
  }
}

class HomeWidget extends StatelessWidget {
  HomeWidget({Key? key}) : super(key: key);

  final items = <Item>[
    Item('Polygons', GridWidget(PolygonPainter())),
  ];

  @override
  Widget build(BuildContext context) {
    return CommonScaffold(
      title: 'Canvas Basics',
      child: Column(
        mainAxisSize: MainAxisSize.max,
        mainAxisAlignment: MainAxisAlignment.center,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: items.map((e) => ItemWidget(e)).toList(),
      ),
    );
  }
}


class ItemWidget extends StatelessWidget {
  final Item item;

  const ItemWidget(this.item, {Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      margin: const EdgeInsets.symmetric(horizontal: 20, vertical: 7),
      child: TextButton(
        style: TextButton.styleFrom(
            backgroundColor: const Color(0xFF158443),
            primary: Colors.white,
            padding: const EdgeInsets.symmetric(vertical: 15)),
        onPressed: () {
          final widget = CommonScaffold(title: item.title, child: item.widget);
          Navigator.of(context)
              .push(MaterialPageRoute<void>(builder: (_) => widget));
        },
        child: Text(
          item.title,
          style: const TextStyle(
            fontSize: 18,
            fontWeight: FontWeight.bold,
          ),
        ),
      ),
    );
  }
}

class Item {
  final String title;
  final Widget widget;

  Item(this.title, this.widget);
}*/
//--------------------------------------------------------


//-------Generate image
/*void main() => runApp(App());

const kCanvasSize = 200.0;

class App extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: ImageGenerator(),
      ),
      debugShowCheckedModeBanner: false,
    );
  }
}

class ImageGenerator extends StatefulWidget {
  final Random rd;
  final int numColors;

  ImageGenerator()
      : rd = Random(),
        numColors = Colors.primaries.length;

  @override
  _ImageGeneratorState createState() => _ImageGeneratorState();
}

class _ImageGeneratorState extends State<ImageGenerator> {
  ByteData? imgBytes; //!

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.max,
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: <Widget>[
          Padding(
            padding: const EdgeInsets.all(12.0),
            child: TextButton(
                onPressed: generateImage, child: const Text('Generate image')),
          ),
          imgBytes != null
              ? Center(
              child: Image.memory(
                Uint8List.view(imgBytes!.buffer),
                width: kCanvasSize,
                height: kCanvasSize,
              ))
              : Container()
        ],
      ),
    );
  }

  void generateImage() async {
    final color = Colors.primaries[widget.rd.nextInt(widget.numColors)];

    final recorder = ui.PictureRecorder(); //!
    final canvas = Canvas(recorder,
        Rect.fromPoints(Offset(0.0, 0.0), Offset(kCanvasSize, kCanvasSize)));

    final stroke = Paint()
      ..color = Colors.grey
      ..style = PaintingStyle.stroke;

    canvas.drawRect(Rect.fromLTWH(0.0, 0.0, kCanvasSize, kCanvasSize), stroke);

    final paint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;

    canvas.drawCircle(
        Offset(
          widget.rd.nextDouble() * kCanvasSize,
          widget.rd.nextDouble() * kCanvasSize,
        ),
        20.0,
        paint);

    final picture = recorder.endRecording(); //!
    final img = await picture.toImage(200, 200);
    final pngBytes = await img.toByteData(format: ImageByteFormat.png); //!

    setState(() {
      imgBytes = pngBytes!; //!?
    });
  }
}*/

//--------------------------------------------------------------------
void main() => runApp(DrawApp());

class DrawApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Draw(),
    );
  }
}

class Draw extends StatefulWidget {


  @override
  _DrawState createState() => _DrawState();
}

const kCanvasSize = 150.0;
ByteData? imgBytes;
class _DrawState extends State<Draw> {
  Color selectedColor = Colors.indigoAccent;
  double strokeWidth = 4.0;
  List<DrawingPoints> points = [];
  bool showBottomList = false;
  double opacity = 1.0;


  @override
  Widget build(BuildContext context) {
    return Scaffold(
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Container(
            padding: const EdgeInsets.only(left: 8.0, right: 8.0),
            decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(50.0),
                color: Colors.greenAccent),
            child: Padding(
              padding: const EdgeInsets.all(8.0),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: <Widget>[
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
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
                          //onPressed: () {
                            //setState(() {
                              //showBottomList = false;

                            //});
                          //}
                          ),
                        imgBytes != null
                          ? Container(
                          color: Colors.yellow,
                          child: Image.memory(
                            Uint8List.view(imgBytes!.buffer),
                            width: kCanvasSize,
                            height: kCanvasSize,
                          ))
                          : Container(color: Colors.yellow,)
                    ],
                  ),
                ],
              ),
            )),
      ),
      body: Row(
          mainAxisAlignment: MainAxisAlignment.center,
        //
        children:[Column(
            crossAxisAlignment: CrossAxisAlignment.center,
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
              child:Container(
                color: Colors.yellow,
                //alignment: const Alignment(0,0),
                //mainAxisAlignment: MainAxisAlignment.center,
                child: CustomPaint(
                  size: Size.square(kCanvasSize), //200
                  painter: DrawingPainter(
                    pointsList: points,
                  ),
                ),
              ),
            ),
          ]),
          imgBytes != null
              ? Container(
              color: Colors.lightBlueAccent,
              child: Image.memory(
                Uint8List.view(imgBytes!.buffer),
                width: kCanvasSize,
                height: kCanvasSize,
              ))
              : Container(color: Colors.lightBlueAccent,),
        ])
    );
  }

  void toImage() async{
    //publish imgBytes
  }

}
//List<DrawingPoints>? globalPointsList;
//List<Offset> globalOffsetPoints = [];
class DrawingPainter extends CustomPainter {
  DrawingPainter({required this.pointsList});
  List<DrawingPoints> pointsList;
  List<Offset> offsetPoints = [];

  @override
  void paint(Canvas canvas, Size size) async{
    final recorder = ui.PictureRecorder(); //!
    final canvas = Canvas(recorder,
        Rect.fromPoints(Offset(-90.0, 0.0), Offset(kCanvasSize, kCanvasSize)));

    for (int i = 0; i < pointsList.length - 1; i++) {
      if (pointsList[i] != null && pointsList[i + 1] != null) {
        canvas.drawLine(pointsList[i].points+Offset(-90,0), pointsList[i + 1].points+Offset(-90,0),
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


//-------------------------------------
