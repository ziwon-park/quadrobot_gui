import 'package:flutter/material.dart';
import 'dart:typed_data';
import '../ros_service.dart';
import '../globals.dart' as globals;

class PointCloudViewer extends StatefulWidget {
  @override
  _PointCloudViewerState createState() => _PointCloudViewerState();
}

class _PointCloudViewerState extends State<PointCloudViewer> {
  List<Offset> points = [];
  late RosService _rosService;
  
  // 뷰어 설정
  final double pointSize = 4.0;  // 점 크기
  final double scale = 20.0;     // 스케일 팩터 (미터를 픽셀로 변환)
  
  @override
  void initState() {
    super.initState();
    _rosService = RosService(globals.full_address);
    _connectToRos();
  }

  Future<void> _connectToRos() async {
    await _rosService.connect();
    print("ROS Connected, subscribing to PointCloud...");
    
    _rosService.subscribeToPointCloud(
      globals.topics['point_cloud']!,
      (pointData) {
        _updatePoints(pointData);
      }
    );
  }

  void _updatePoints(List<double> rawPoints) {
    List<Offset> newPoints = [];
    
    // x, y, z 좌표를 2D 포인트로 변환
    for (int i = 0; i < rawPoints.length; i += 3) {
      if (i + 2 < rawPoints.length) {
        // x와 y 좌표만 사용 (z는 무시)
        double x = rawPoints[i];
        double y = rawPoints[i + 1];
        
        newPoints.add(Offset(x, y));
      }
    }
    
    setState(() {
      points = newPoints;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      color: Colors.black,
      child: CustomPaint(
        painter: PointCloudPainter(
          points: points,
          pointSize: pointSize,
          scale: scale,
        ),
        size: Size.infinite,
      ),
    );
  }
}

class PointCloudPainter extends CustomPainter {
  final List<Offset> points;
  final double pointSize;
  final double scale;
  
  PointCloudPainter({
    required this.points,
    required this.pointSize,
    required this.scale,
  });
  
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.green
      ..style = PaintingStyle.fill;
      
    // 캔버스를 중앙으로 이동
    canvas.translate(size.width / 2, size.height / 2);
    
    // 모든 포인트 그리기
    for (var point in points) {
      canvas.drawCircle(
        Offset(point.dx * scale, point.dy * scale),
        pointSize,
        paint,
      );
    }
    
    // 좌표축 그리기 (옵션)
    final axisPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.0;
      
    canvas.drawLine(
      Offset(-size.width/2, 0),
      Offset(size.width/2, 0),
      axisPaint,
    );
    canvas.drawLine(
      Offset(0, -size.height/2),
      Offset(0, size.height/2),
      axisPaint,
    );
  }
  
  @override
  bool shouldRepaint(covariant PointCloudPainter oldDelegate) {
    return true;  // 항상 다시 그리기
  }
}