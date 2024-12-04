import 'package:flutter/material.dart';
import '../ros_service.dart';
import 'dart:convert';
import 'package:get/get.dart';
import 'dart:async';
import '../globals.dart' as globals;
import 'package:fl_chart/fl_chart.dart';

class JoystickController extends GetxController {
  var center = Offset(0, 0).obs;
  var movement = Offset(0, 0).obs;
  double radius = 100;

  var maxLinVel = 0.1.obs;
  var maxRotVel = 0.05.obs;

  var linearVelocities = <FlSpot>[].obs;
  var angularVelocities = <FlSpot>[].obs;
  var currentTime = 0.0.obs;

  updatePosition(DragUpdateDetails position) {
    movement.value = position.localPosition - center.value;
    double distance = movement.value.distance;
    if (distance > radius) {
      movement.value = movement.value * (radius / distance);
    }
  }

  void addVelocityData(double linear, double angular) {
    final time = currentTime.value;
    
    linearVelocities.removeWhere((spot) => spot.x < time - 3.0);
    angularVelocities.removeWhere((spot) => spot.x < time - 3.0);
    
    linearVelocities.add(FlSpot(time, linear));
    angularVelocities.add(FlSpot(time, angular));
    
    currentTime.value += 0.1; 
  }
}

class ControllerBlock extends StatefulWidget {
  @override
  _ControllerBlockState createState() => _ControllerBlockState();
}

class _ControllerBlockState extends State<ControllerBlock> {
  late RosService rosService;
  late JoystickController controller;
  Timer? timer;
  Timer? plotTimer;

  @override
  void initState() {
    super.initState();
    rosService = RosService(globals.full_address);
    controller = Get.put(JoystickController());
    _connectToRos();
    
    plotTimer = Timer.periodic(Duration(seconds: 1), (Timer t) {
      if (mounted) {
        setState(() {});
      }
    });
  }

  Future<void> _connectToRos() async {
    await rosService.connect();
    timer = Timer.periodic(Duration(milliseconds: 100), (Timer t) => sendCmd());
  }

  void sendCmd() {
    double linear = -controller.movement.value.dy / controller.radius * controller.maxLinVel.value;
    double angular = -controller.movement.value.dx / controller.radius * controller.maxRotVel.value;

    controller.addVelocityData(linear, angular);

    var message = {
      'op': 'publish',
      'topic': '/cmd/velocity',
      'msg': {
        'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
      },
    };
    rosService.publish(json.encode(message));
  }

  Widget buildVelocityChart(String title, List<FlSpot> spots, Color color) {
    return Container(
      height: 200,
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          Text(title),
          Expanded(
            child: LineChart(
              LineChartData(
                minY: -1.0,
                maxY: 1.0,
                minX: controller.currentTime.value - 3.0,
                maxX: controller.currentTime.value,
                gridData: FlGridData(show: true),
                titlesData: FlTitlesData(
                  rightTitles: AxisTitles(sideTitles: SideTitles(showTitles: false)),
                  topTitles: AxisTitles(sideTitles: SideTitles(showTitles: false)),
                  bottomTitles: AxisTitles(sideTitles: SideTitles(showTitles: false)),
                ),
                borderData: FlBorderData(show: true),
                lineBarsData: [
                  LineChartBarData(
                    spots: spots,
                    isCurved: true,
                    color: color,
                    dotData: FlDotData(show: false),
                    belowBarData: BarAreaData(show: false),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  void resetValues() {
    controller.maxLinVel.value = 0.1;
    controller.maxRotVel.value = 0.05;
  }

  @override
  void dispose() {
    timer?.cancel();
    plotTimer?.cancel();
    rosService.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4.0,
      child: Container(
        padding: EdgeInsets.all(16.0),
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Expanded(
              flex: 2,
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Obx(() => buildVelocityChart(
                        'Linear Velocity (m/s)',
                        controller.linearVelocities,
                        Colors.blue,
                      )),
                  SizedBox(height: 16),
                  Obx(() => buildVelocityChart(
                        'Angular Velocity (rad/s)',
                        controller.angularVelocities,
                        Colors.red,
                      )),
                ],
              ),
            ),
            Expanded(
              flex: 3,
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Text(
                    'Robot Control',
                    style: TextStyle(
                      fontSize: 18,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  SizedBox(height: 24.0),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    crossAxisAlignment: CrossAxisAlignment.center,
                    children: [
                      Column(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Obx(() => Text(
                                '${controller.maxRotVel.value.toStringAsFixed(2)} rad/s',
                                style: TextStyle(fontSize: 14),
                              )),
                          RotatedBox(
                            quarterTurns: 3,
                            child: Obx(() => Slider(
                                  value: controller.maxRotVel.value,
                                  min: 0,
                                  max: 1.0,
                                  divisions: 20,
                                  onChanged: (value) {
                                    controller.maxRotVel.value = value;
                                  },
                                )),
                          ),
                        ],
                      ),
                      SizedBox(width: 16),
                      GestureDetector(
                        onPanStart: (details) {
                          controller.center.value = details.localPosition;
                          controller.movement.value = Offset(0, 0);
                        },
                        onPanUpdate: (details) {
                          controller.updatePosition(details);
                        },
                        onPanEnd: (details) {
                          controller.movement.value = Offset(0, 0);
                        },
                        child: Obx(() => Container(
                              width: controller.radius * 2,
                              height: controller.radius * 2,
                              decoration: BoxDecoration(
                                color: Colors.grey[200],
                                shape: BoxShape.circle,
                              ),
                              child: Center(
                                child: Transform.translate(
                                  offset: controller.movement.value,
                                  child: Container(
                                    width: controller.radius,
                                    height: controller.radius,
                                    decoration: BoxDecoration(
                                      color: Colors.blue,
                                      shape: BoxShape.circle,
                                    ),
                                  ),
                                ),
                              ),
                            )),
                      ),
                      SizedBox(width: 16),
                      Column(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Obx(() => Text(
                                '${controller.maxLinVel.value.toStringAsFixed(2)} m/s',
                                style: TextStyle(fontSize: 14),
                              )),
                          RotatedBox(
                            quarterTurns: 3,
                            child: Obx(() => Slider(
                                  value: controller.maxLinVel.value,
                                  min: 0,
                                  max: 1.0,
                                  divisions: 20,
                                  onChanged: (value) {
                                    controller.maxLinVel.value = value;
                                  },
                                )),
                          ),
                        ],
                      ),
                    ],
                  ),
                  SizedBox(height: 24.0),
                  ElevatedButton(
                    onPressed: resetValues,
                    child: Text('Reset Values'),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
