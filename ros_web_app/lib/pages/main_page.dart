import 'package:flutter/material.dart';
import '../globals.dart' as globals;

import '../ros_service.dart';
import './camera.dart';
import './controller.dart';

class MainPage extends StatefulWidget {
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Robot Control Panel'),
      ),
      body: SingleChildScrollView(
        padding: EdgeInsets.all(16.0),
        child: Column(
          children: [
            CameraViewer(),
            SizedBox(height: 16.0),
            ControllerBlock(),
          ],
        ),
      ),
    );
  }
}