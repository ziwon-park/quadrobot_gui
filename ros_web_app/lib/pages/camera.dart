import 'package:flutter/material.dart';
import 'dart:typed_data';
import '../ros_service.dart';
import '../globals.dart' as globals;

class CameraViewer extends StatefulWidget {
  @override
  _CameraViewerState createState() => _CameraViewerState();
}

class _CameraViewerState extends State<CameraViewer> {
  final ValueNotifier<Uint8List?> _imageNotifier = ValueNotifier<Uint8List?>(null);
  late RosService _rosService;

  @override
  void initState() {
    super.initState();
    _rosService = RosService(globals.full_address);
    _connectToRos();
  }

  Future<void> _connectToRos() async {
    await _rosService.connect();
    _rosService.subscribeImage(
      '/camera_face/color/image_raw/compressed', 
      (Uint8List data, int size) {
        _imageNotifier.value = data;
      }
    );
  }


  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4.0,
      child: Container(
        height: 300,
        padding: EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Camera View',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 8.0),
            Expanded(
              child: ValueListenableBuilder<Uint8List?>(
                valueListenable: _imageNotifier,
                builder: (context, imageData, child) {
                  return imageData != null
                    ? Image.memory(
                        imageData,
                        fit: BoxFit.contain,
                        gaplessPlayback: true,
                      )
                    : Center(
                        child: CircularProgressIndicator(),
                      );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    _imageNotifier.dispose();
    _rosService.disconnect();
    super.dispose();
  }
}