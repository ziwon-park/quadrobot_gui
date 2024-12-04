import 'package:flutter/material.dart';
import '../globals.dart' as globals;

import '../ros_service.dart';
import './camera.dart';
import './controller.dart';
import './map.dart';
import './communication.dart';

class MainPage extends StatefulWidget {
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final isWideScreen = screenWidth > 1200;  // 화면 너비가 1200px 이상일 때 와이드 스크린으로 간주

    return Scaffold(
      appBar: AppBar(
        title: Text('Robot Control Panel'),
      ),
      body: SingleChildScrollView(
        padding: EdgeInsets.all(16.0),
        child: isWideScreen ? _buildWideLayout() : _buildNarrowLayout(),
      ),
    );
  }

  Widget _buildWideLayout() {
    return Center(
      child: Container(
        constraints: BoxConstraints(maxWidth: 1600),
        child: Column(
          children: [
            // First row
            Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Expanded(
                  child: Padding(
                    padding: EdgeInsets.all(8.0),
                    child: MapViewer(),
                  ),
                ),
                Expanded(
                  child: Padding(
                    padding: EdgeInsets.all(8.0),
                    child: CameraViewer(),
                  ),
                ),
              ],
            ),
            SizedBox(height: 16.0),
            // Second row
            Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Expanded(
                  child: Padding(
                    padding: EdgeInsets.all(8.0),
                    child: CommunicationPanel(),
                  ),
                ),
                Expanded(
                  child: Padding(
                    padding: EdgeInsets.all(8.0),
                    child: ControllerBlock(),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildNarrowLayout() {
    return Column(
      children: [
        Padding(
          padding: EdgeInsets.all(8.0),
          child: MapViewer(),
        ),
        SizedBox(height: 16.0),
        Padding(
          padding: EdgeInsets.all(8.0),
          child: CameraViewer(),
        ),
        SizedBox(height: 16.0),
        Padding(
          padding: EdgeInsets.all(8.0),
          child: CommunicationPanel(),
        ),
        SizedBox(height: 16.0),
        Padding(
          padding: EdgeInsets.all(8.0),
          child: ControllerBlock(),
        ),
      ],
    );
  }
}
