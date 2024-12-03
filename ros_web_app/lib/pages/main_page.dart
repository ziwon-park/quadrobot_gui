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
            MapViewer(),
            SizedBox(height: 16.0),
            CommunicationPanel(),
            SizedBox(height: 16.0),
            ControllerBlock(),
          ],
        ),
      ),
    );
  }
}

// import 'package:flutter/material.dart';
// import '../ros_service.dart';
// import 'package:flutter/material.dart';
// import './camera.dart';
// // import './map.dart';
// // import './point_cloud.dart';
// // import './controller.dart';
// // import './communication.dart';

// class MainPage extends StatefulWidget {
//   @override
//   _MainPageState createState() => _MainPageState();
// }

// class _MainPageState extends State<MainPage> {
//   @override
//   Widget build(BuildContext context) {
//     final screenWidth = MediaQuery.of(context).size.width;
//     final isWideScreen = screenWidth > 1200;

//     return Scaffold(
//       body: SingleChildScrollView(
//         child: Padding(
//           padding: EdgeInsets.all(16.0),
//           child: isWideScreen 
//             ? _buildWideLayout()
//             : _buildNarrowLayout(),
//         ),
//       ),
//     );
//   }

//   Widget _buildWideLayout() {
//     return Column(
//       children: [
//         // First row: Camera and Map
//         Row(
//           children: [
//             Expanded(
//               child: Padding(
//                 padding: EdgeInsets.all(8.0),
//                 child: CameraViewer(),
//               ),
//             ),
//             Expanded(
//               child: Padding(
//                 padding: EdgeInsets.all(8.0),
//                 child: MapViewer(),
//               ),
//             ),
//           ],
//         ),
//         // Second row: PointCloud and Controller
//         Row(
//           children: [
//             Expanded(
//               child: Padding(
//                 padding: EdgeInsets.all(8.0),
//                 child: PointCloudViewer(),
//               ),
//             ),
//             Expanded(
//               child: Padding(
//                 padding: EdgeInsets.all(8.0),
//                 child: ControllerViewer(),
//               ),
//             ),
//           ],
//         ),
//         // Third row: Communication
//         Padding(
//           padding: EdgeInsets.all(8.0),
//           child: CommunicationViewer(),
//         ),
//       ],
//     );
//   }

//   Widget _buildNarrowLayout() {
//     return Column(
//       children: [
//         Padding(
//           padding: EdgeInsets.all(8.0),
//           child: CameraViewer(),
//         ),
//         Padding(
//           padding: EdgeInsets.all(8.0),
//           child: MapViewer(),
//         ),
//         Padding(
//           padding: EdgeInsets.all(8.0),
//           child: PointCloudViewer(),
//         ),
//         Padding(
//           padding: EdgeInsets.all(8.0),
//           child: ControllerViewer(),
//         ),
//         Padding(
//           padding: EdgeInsets.all(8.0),
//           child: CommunicationViewer(),
//         ),
//       ],
//     );
//   }
// }
