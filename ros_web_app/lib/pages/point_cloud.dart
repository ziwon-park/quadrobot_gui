import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import 'package:three_dart/three_dart.dart' as three;
import 'package:flutter_gl/flutter_gl.dart';
import 'dart:typed_data';

class _PointCloudViewerState extends State<PointCloudViewer> {
  late Scene scene;
  late PerspectiveCamera camera;
  late WebGLRenderer renderer;
  late OrbitControls controls;
  Points? pointCloud;
  final ValueNotifier<Uint8List?> _pointCloudNotifier = ValueNotifier<Uint8List?>(null);
  late RosService _rosService;
  
  @override
  void initState() {
    super.initState();
    _rosService = RosService(globals.full_address);
    _initThree();
    _connectToRos();
  }

  void _initThree() async {
    scene = Scene();
    
    camera = PerspectiveCamera(
      75, 
      MediaQuery.of(context).size.width / MediaQuery.of(context).size.height,
      0.1,
      1000
    );
    camera.position.z = 5;
    
    var gl = await FlutterGl.initialize();
    renderer = WebGLRenderer(gl);
    renderer.setSize(
      MediaQuery.of(context).size.width.toInt(),
      MediaQuery.of(context).size.height.toInt()
    );
    
    controls = OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    
    var ambientLight = AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);
    
    var directionalLight = DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(0, 1, 0);
    scene.add(directionalLight);
    
    _animate();
  }

  void _animate() {
    controls.update();
    renderer.render(scene, camera);
    Future.delayed(Duration(milliseconds: 16), _animate);
  }

  void _updatePointCloud(List<double> points) {
    if (pointCloud != null) {
      scene.remove(pointCloud!);
    }
    
    var geometry = BufferGeometry();
    var positions = Float32Array.fromList(points);
    
    geometry.setAttribute(
      'position',
      BufferAttribute(positions, 3)  
    );
    
    var material = PointsMaterial(
      color: Color(0x00ff00),  
      size: 0.02,
      sizeAttenuation: true
    );
    
    pointCloud = Points(geometry, material);
    scene.add(pointCloud!);
  }

  Future<void> _connectToRos() async {
    await _rosService.connect();
    print("ROS Connected, subscribing to PointCloud...");
    
    _rosService.subscribeToPointCloud(
      globals.topics['point_cloud']!,
      (points) {
        _updatePointCloud(points);
      }
    );
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      width: MediaQuery.of(context).size.width,
      height: MediaQuery.of(context).size.height,
      child: renderer.domElement
    );
  }

  @override
  void dispose() {
    controls.dispose();
    renderer.dispose();
    super.dispose();
  }
}