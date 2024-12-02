import 'package:flutter/material.dart';
import '../ros_service.dart';
import 'dart:convert';
import 'package:get/get.dart';
import 'dart:async';
import '../globals.dart' as globals;

class JoystickController extends GetxController {
  var center = Offset(0, 0).obs;
  var movement = Offset(0, 0).obs;
  double radius = 100;

  var maxLinVel = 0.1.obs;
  var maxRotVel = 0.05.obs;

  updatePosition(DragUpdateDetails position) {
    movement.value = position.localPosition - center.value;
    double distance = movement.value.distance;
    if (distance > radius) {
      movement.value = movement.value * (radius / distance);
    }
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

  @override
  void initState() {
    super.initState();
    rosService = RosService(globals.full_address);
    controller = Get.put(JoystickController());
    _connectToRos();
  }

  Future<void> _connectToRos() async {
    await rosService.connect();
    timer = Timer.periodic(Duration(milliseconds: 100), (Timer t) => sendCmd());
  }

  void sendCmd() {
    double linear = -controller.movement.value.dy / controller.radius * controller.maxLinVel.value;
    double angular = -controller.movement.value.dx / controller.radius * controller.maxRotVel.value;

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

  void resetValues() {
    controller.maxLinVel.value = 0.1;
    controller.maxRotVel.value = 0.05;
  }

  @override
  void dispose() {
    timer?.cancel();
    rosService.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4.0,
      child: Container(
        padding: EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Robot Control',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 16.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
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
            SizedBox(height: 16.0),
            Center(
              child: ElevatedButton(
                onPressed: resetValues,
                child: Text('Reset Values'),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
