import 'package:flutter/material.dart';
import 'dart:typed_data';
import '../../ros_service.dart';
import '../../globals.dart' as globals;

class GazeboImageWidget extends StatefulWidget {
  @override
  _GazeboImageWidgetState createState() => _GazeboImageWidgetState();
}

class _GazeboImageWidgetState extends<GazeboImageWidget>{
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: SingleChildScrollView(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              ImageLoaderWidget(topic: '/camera_face/color/image_raw/compressed'),
              SizedBox(height: 10),
              ImageLoaderWidget(topic: '/camera_left/color/image_raw/compressed'),
              SizedBox(height: 10),
              ImageLoaderWidget(topic: '/camera_rearDown/color/image_raw/compressed'),
              SizedBox(height: 10),
              ImageLoaderWidget(topic: '/camera_right/color/image_raw/compressed'),
            ],
          ),
        ),
      ),
    );
  }}


class ImageLoaderWidget extends StatefulWidget {
  final String topic;

  ImageLoaderWidget({Key? key, required this.topic}) : super(key: key);

  @override
  _ImageLoaderWidgetState createState() => _ImageLoaderWidgetState();
}

class _ImageLoaderWidgetState extends State<ImageLoaderWidget> {
  Uint8List? imageBytes;
  DateTime? evaluationStartTime;
  int receivedDataBytes = 0; // 수신된 데이터 양
  int receivedImagesCount = 0; // 수신된 이미지 수

  @override
  void initState() {
    super.initState();
    final rosService = RosService(globals.full_address);
    rosService.connect().then((_) {
      rosService.subscribeImage(widget.topic, (Uint8List data, int dataSize) {
        if (mounted) {
          setState(() {
            imageBytes = data;
            receivedDataBytes += dataSize;
            receivedImagesCount += 1;
          });
        }
      });
    });
  }

  void evaluateCommunication() {
    evaluationStartTime = DateTime.now();
    receivedDataBytes = 0;
    receivedImagesCount = 0;

    Future.delayed(Duration(seconds: 3), () {
      final evaluationEndTime = DateTime.now();
      final durationInSeconds = evaluationEndTime.difference(evaluationStartTime!).inSeconds;
      final dataRate = receivedDataBytes / durationInSeconds;

      showDialog(
        context: context,
        builder: (BuildContext context) {
          return AlertDialog(
            title: Text('Communication Evaluation'),
            content: Text('Received Images: $receivedImagesCount\n'
                'Total Data: $receivedDataBytes bytes\n'
                'Duration: $durationInSeconds seconds\n'
                'Data Rate: $dataRate bytes/s'),
            actions: <Widget>[
              TextButton(
                child: Text('Close'),
                onPressed: () {
                  Navigator.of(context).pop();
                },
              ),
            ],
          );
        },
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          padding: EdgeInsets.symmetric(vertical: 5, horizontal: 10),
          decoration: BoxDecoration(
            color: Colors.grey,
            borderRadius: BorderRadius.circular(20),
          ),
          child: Text(
            widget.topic,
            style: TextStyle(fontSize: 12, color: Colors.white, fontWeight: FontWeight.bold),
          ),
        ),
        SizedBox(height: 10),
        imageBytes != null
            ? Image.memory(imageBytes!, gaplessPlayback: true)
            : CircularProgressIndicator(),
        SizedBox(height: 5),
        ElevatedButton(
          onPressed: () => evaluateCommunication(),
          child: Text('Evaluate Communication'),
        ),
      ],
    );
  }
}