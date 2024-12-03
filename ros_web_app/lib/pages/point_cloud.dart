import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
import 'package:three_dart/three_dart.dart' as three;
import 'package:flutter_gl/flutter_gl.dart';
import 'package:flutter_gl/native-array/index.dart';
import 'dart:typed_data';
import '../globals.dart' as globals;
import '../ros_service.dart';

class PointCloudViewer extends StatefulWidget {
  const PointCloudViewer({Key? key}) : super(key: key);

  @override
  _PointCloudViewerState createState() => _PointCloudViewerState();
}

class _PointCloudViewerState extends State<PointCloudViewer> {
  late FlutterGlPlugin three3dRender;
  three.WebGLRenderer? renderer;
  three.Scene? scene;
  three.PerspectiveCamera? camera;
  three.Points? points;  // nullable로 변경
  RosService? _rosService;
  
  bool hasData = false;
  int pointCount = 0;
  
  Size? screenSize;
  int? fboId;
  double width = 600;
  double height = 400;
  bool disposed = false;
  bool isInitialized = false;

  @override
  void initState() {
    super.initState();
    three3dRender = FlutterGlPlugin();
    _initializeViewer();
  }

  Future<void> _initializeViewer() async {
    await initPlatformState();
    _rosService = RosService(globals.full_address);
    await _connectToRos();
  }

  Future<void> _connectToRos() async {
    await _rosService?.connect();
    print("ROS Connected, subscribing to point cloud...");
    _rosService?.subscribeToPointCloud(
      '/os_cloud_node/points/downsampled',
      (List<double> points) {
        print("Received point cloud data: ${points.length} values");
        if (isInitialized) {
          updatePointCloud(points);
        }
      }
    );
  }

  Future<void> initPlatformState() async {
    Map<String, dynamic> options = {
      "antialias": true,
      "alpha": false,
      "width": width.toInt(),
      "height": height.toInt(),
      "dpr": 1.0
    };

    await three3dRender.initialize(options: options);
    await initScene();
    setState(() {
      isInitialized = true;
    });
  }

  Future<void> initScene() async {
    scene = three.Scene();
    
    camera = three.PerspectiveCamera(60, width / height, 0.1, 1000);
    camera?.position.set(0, 0, 5);
    camera?.lookAt(three.Vector3(0, 0, 0));

    Map<String, dynamic> renderProps = {
      "canvas": three3dRender.element,
      "context": three3dRender.gl,
      "antialias": true
    };
    renderer = three.WebGLRenderer(renderProps);
    renderer?.setSize(width, height, true);
    
    var gridHelper = three.GridHelper(10, 10);
    scene?.add(gridHelper);
    
    var axesHelper = three.AxesHelper(5);
    scene?.add(axesHelper);
    
    var geometry = three.BufferGeometry();
    var material = three.PointsMaterial({
      "size": 0.02,
      "sizeAttenuation": true,
      "color": three.Color(0x00ff00),
    });
    
    points = three.Points(geometry, material);
    // points?.rotation.x = -three.Math.PI / 2;
    scene?.add(points!);

    animate();
  }

  void updatePointCloud(List<double> pointsData) {
    if (disposed || points == null) return;

    try {
      Float32Array float32Array = Float32Array(pointsData.length);
      for (int i = 0; i < pointsData.length; i++) {
        float32Array[i] = pointsData[i];
      }

      var positions = three.Float32BufferAttribute(float32Array, 3);
      var geometry = three.BufferGeometry();
      geometry.setAttribute('position', positions);

      points?.geometry = geometry;
      
      setState(() {
        hasData = true;
        pointCount = pointsData.length ~/ 3;
      });
      
      print("Updated point cloud with ${pointCount} points");
    } catch (e) {
      print("Error updating point cloud: $e");
    }
  }

  void animate() {
    if (disposed) return;
    render();
    Future.delayed(const Duration(milliseconds: 16), animate);
  }

  void render() {
    if (disposed || !isInitialized || scene == null || camera == null) return;

    final _gl = three3dRender.gl;
    renderer?.render(scene!, camera!);
    _gl.flush();

    if (!kIsWeb) {
      three3dRender.updateTexture(fboId!);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      width: width,
      height: height,
      child: Stack(
        children: [
          if (three3dRender.isInitialized)
            HtmlElementView(viewType: three3dRender.textureId!.toString()),
          Positioned(
            top: 10,
            right: 10,
            child: Container(
              padding: EdgeInsets.all(8),
              color: Colors.black54,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.end,
                children: [
                  Text(
                    'Point Cloud Viewer',
                    style: TextStyle(color: Colors.white),
                  ),
                  if (hasData) Text(
                    'Points: $pointCount',
                    style: TextStyle(color: Colors.white),
                  ),
                ],
              ),
            ),
          ),
          if (!three3dRender.isInitialized)
            const Center(child: CircularProgressIndicator()),
        ],
      ),
    );
  }

  @override
  void dispose() {
    disposed = true;
    three3dRender.dispose();
    _rosService?.disconnect();
    super.dispose();
  }
}