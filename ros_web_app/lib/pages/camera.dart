import 'package:flutter/material.dart';
import 'dart:typed_data';
import '../ros_service.dart';
import '../globals.dart' as globals;

class CameraViewer extends StatefulWidget {
  @override
  _CameraViewerState createState() => _CameraViewerState();
}

class _CameraViewerState extends State<CameraViewer> {
  Uint8List? _imageData;
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
      '/camera/image_raw/compressed',  // 실제 카메라 토픽으로 수정 필요
      (Uint8List data, int size) {
        setState(() {
          _imageData = data;
        });
      }
    );
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4.0,
      child: Container(
        height: 300,  // 높이는 상황에 맞게 조정
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
              child: _imageData != null
                ? Image.memory(
                    _imageData!,
                    fit: BoxFit.contain,
                  )
                : Center(
                    child: CircularProgressIndicator(),
                  ),
            ),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    _rosService.disconnect();
    super.dispose();
  }
}