import 'package:flutter/material.dart';
import '../ros_service.dart';
import 'dart:async';

class CommunicationPanel extends StatefulWidget {
  @override
  _CommunicationPanelState createState() => _CommunicationPanelState();
}

class TopicStats {
  String name;
  double hz = 0;
  double latency = 0;
  int messageCount = 0;
  DateTime? lastMessageTime;

  TopicStats(this.name);

  void updateStats() {
    if (lastMessageTime != null) {
      final now = DateTime.now();
      final duration = now.difference(lastMessageTime!);
      latency = duration.inMilliseconds.toDouble();
      
      // Calculate hz over a 1-second window
      if (duration.inSeconds >= 1) {
        hz = messageCount / duration.inSeconds;
        messageCount = 0;
        lastMessageTime = now;
      }
    }
  }

  void onMessageReceived() {
    final now = DateTime.now();
    messageCount++;
    if (lastMessageTime == null) {
      lastMessageTime = now;
    }
    updateStats();
  }
}

class _CommunicationPanelState extends State<CommunicationPanel> {
  final Map<String, TopicStats> topicStats = {};
  Timer? _updateTimer;
  final List<String> monitoredTopics = [
    '/go1_gazebo/camera/color/image_raw/compressed',
    '/map',
  ];

  @override
  void initState() {
    super.initState();
    _initializeTopics();
    _updateTimer = Timer.periodic(Duration(milliseconds: 100), (timer) {
      setState(() {
        topicStats.values.forEach((stats) => stats.updateStats());
      });
    });
  }

  void _initializeTopics() {
    for (var topic in monitoredTopics) {
      topicStats[topic] = TopicStats(topic);
      RosService rosService = RosService('ws://localhost:9090');
      rosService.connect().then((_) {
        rosService.subscribeTopic(topic, (data) {
          if (mounted) {
            setState(() {
              topicStats[topic]?.onMessageReceived();
            });
          }
        });
      });
    }
  }

  @override
  void dispose() {
    _updateTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Topic Statistics',
              style: Theme.of(context).textTheme.titleLarge,
            ),
            SizedBox(height: 16),
            ...topicStats.entries.map((entry) {
              return Padding(
                padding: EdgeInsets.symmetric(vertical: 8.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Topic: ${entry.key}',
                      style: TextStyle(fontWeight: FontWeight.bold),
                    ),
                    Row(
                      children: [
                        Expanded(
                          child: Text('Hz: ${entry.value.hz.toStringAsFixed(2)}'),
                        ),
                        Expanded(
                          child: Text(
                            'Latency: ${entry.value.latency.toStringAsFixed(2)}ms',
                          ),
                        ),
                      ],
                    ),
                    Divider(),
                  ],
                ),
              );
            }).toList(),
          ],
        ),
      ),
    );
  }
}