import 'package:flutter/material.dart';
import 'dart:typed_data';
import 'dart:ui' as ui;
import '../ros_service.dart';
import '../globals.dart' as globals;

class MapViewer extends StatefulWidget {
  @override
  _MapViewerState createState() => _MapViewerState();
}

class _MapViewerState extends State<MapViewer> {
  ui.Image? mapImage;
  double mapResolution = 0.05; // meters per pixel
  int mapWidth = 0;
  int mapHeight = 0;
  List<int>? mapData;
  late RosService _rosService;

  @override
  void initState() {
    super.initState();
    _rosService = RosService(globals.full_address);
    _connectToRos();
  }

  Future<void> _connectToRos() async {
    await _rosService.connect();
    print("ROS Connected, subscribing to map...");

    _rosService.subscribeToMap('/map', (data, width, height, resolution) {
      setState(() {
        mapData = data;
        mapWidth = width;
        mapHeight = height;
        mapResolution = resolution;
        _createMapImage();
      });
    });
  }

  Future<void> _createMapImage() async {
    if (mapData == null || mapWidth == 0 || mapHeight == 0) return;

    final bytes = Uint8List(mapWidth * mapHeight * 4);
    int pixelIndex = 0;

    for (int i = 0; i < mapData!.length; i++) {
      int value = mapData![i];
      int color;
      
      if (value == -1) {  // Unknown
        color = 0xFF808080;  // Gray
      } else if (value == 100) {  // Occupied
        color = 0xFF000000;  // Black
      } else {  // Free
        color = 0xFFFFFFFF;  // White
      }

      bytes[pixelIndex++] = (color >> 16) & 0xFF;  // R
      bytes[pixelIndex++] = (color >> 8) & 0xFF;   // G
      bytes[pixelIndex++] = color & 0xFF;          // B
      bytes[pixelIndex++] = 0xFF;                  // A
    }

    final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(bytes);
    final ui.ImageDescriptor descriptor = ui.ImageDescriptor.raw(
      buffer,
      width: mapWidth,
      height: mapHeight,
      pixelFormat: ui.PixelFormat.rgba8888,
    );

    mapImage = await descriptor.instantiateCodec().then((codec) => codec.getNextFrame())
        .then((frame) => frame.image);

    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4.0,
      child: Container(
        height: 600,
        padding: EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Map View',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 16.0),
            Expanded(
              child: mapImage == null
                  ? Center(child: CircularProgressIndicator())
                  : InteractiveViewer(
                      boundaryMargin: EdgeInsets.all(20.0),
                      minScale: 0.1,
                      maxScale: 4.0,
                      child: CustomPaint(
                        painter: MapPainter(mapImage!),
                        size: Size(mapWidth.toDouble(), mapHeight.toDouble()),
                      ),
                    ),
            ),
          ],
        ),
      ),
    );
  }
}

class MapPainter extends CustomPainter {
  final ui.Image image;

  MapPainter(this.image);

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawImage(image, Offset.zero, Paint());
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}