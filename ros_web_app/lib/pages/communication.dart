import 'package:flutter/material.dart';
import '../ros_service.dart';
import 'dart:async';
import 'dart:math' as math;
import 'package:fl_chart/fl_chart.dart';

class CommunicationPanel extends StatefulWidget {
  @override
  _CommunicationPanelState createState() => _CommunicationPanelState();
}

class TopicStats {
  String name;
  double hz = 0;
  double latency = 0;
  double bandwidth = 0;  // MB/s
  int messageCount = 0;
  DateTime? lastMessageTime;
  int lastMessageSize = 0;

  List<double> recentLatencies = [];
  List<DateTime> messageTimestamps = [];

  static const int LATENCY_WINDOW_SIZE = 20;
  static const int HZ_WINDOW_DURATION = 1000;

  TopicStats(this.name);

  void updateStats() {
    final now = DateTime.now();
    
    messageTimestamps = messageTimestamps.where(
      (timestamp) => now.difference(timestamp).inMilliseconds < HZ_WINDOW_DURATION
    ).toList();
    
    hz = messageTimestamps.length.toDouble();
    
    if (recentLatencies.isNotEmpty) {
      latency = recentLatencies.reduce((a, b) => a + b) / recentLatencies.length;
    }
    
    if (lastMessageTime != null) {
      final duration = now.difference(lastMessageTime!);
      if (duration.inSeconds >= 1) {
        bandwidth = (lastMessageSize * messageCount) / (1024 * 1024); // MB/s
        messageCount = 0;
        lastMessageTime = now;
      }
    }
  }



  void onMessageReceived(int messageSize) {
    final now = DateTime.now();
    
    messageTimestamps.add(now);
    
    if (lastMessageTime != null) {
      final currentLatency = now.difference(lastMessageTime!).inMilliseconds.toDouble();
      recentLatencies.add(currentLatency);
      
      if (recentLatencies.length > LATENCY_WINDOW_SIZE) {
        recentLatencies.removeAt(0);
      }
    }
    
    messageCount++;
    lastMessageSize = messageSize;
    lastMessageTime = now;
    
    updateStats();
  }

}


class _CommunicationPanelState extends State<CommunicationPanel> {
  final Map<String, TopicStats> topicStats = {};
  Timer? _updateTimer;
  final List<String> monitoredTopics = [
    '/go1_gazebo/camera/color/image_raw/compressed',
    '/cmd/velocity',
  ];

  final List<Color> colors = [
    Colors.blue,
    Colors.green,
    Colors.orange,
  ];

  double getMaxValue(List<double> values) {
    if (values.isEmpty) return 10.0;
    return values.reduce((a, b) => math.max(a, b));
  }

  Widget getTitleWidget(double value, TitleMeta meta) {
    final topics = topicStats.keys.toList();
    if (value.toInt() >= topics.length) return const Text('');
    return Padding(
      padding: const EdgeInsets.only(top: 8.0),
      child: Text(
        topics[value.toInt()].split('/').last,
        style: const TextStyle(fontSize: 12),
      ),
    );
  }

  BarChartData createChartData(List<double> values, String title, {double? maxYValue}) {
    return BarChartData(
      alignment: BarChartAlignment.spaceAround,
      maxY: maxYValue ?? getMaxValue(values) * 1.2,
      titlesData: FlTitlesData(
        show: true,
        bottomTitles: AxisTitles(
          sideTitles: SideTitles(
            showTitles: true,
            getTitlesWidget: getTitleWidget,
            reservedSize: 42,
          ),
        ),
        leftTitles: AxisTitles(
          axisNameWidget: Text(title),
          sideTitles: SideTitles(
            showTitles: true,
            reservedSize: 42,
            getTitlesWidget: (value, meta) {
              return Text(value.toStringAsFixed(1));
            },
          ),
        ),
        topTitles: AxisTitles(
          sideTitles: SideTitles(showTitles: false),
        ),
        rightTitles: AxisTitles(
          sideTitles: SideTitles(showTitles: false),
        ),
      ),
      gridData: FlGridData(show: true),
      barGroups: List.generate(
        topicStats.length,
        (index) => BarChartGroupData(
          x: index,
          barRods: [
            BarChartRodData(
              toY: values[index],
              color: colors[index % colors.length],
              width: 20,
              borderRadius: BorderRadius.zero,
            ),
          ],
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final List<double> hzValues = topicStats.values.map((e) => e.hz).toList();
    final List<double> latencyValues = topicStats.values.map((e) => e.latency).toList();
    final List<double> bandwidthValues = topicStats.values.map((e) => e.bandwidth).toList();

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
            SizedBox(height: 24),
            
            // Hz Chart
            // AspectRatio(
            //   aspectRatio: 2,
            //   child: Padding(
            //     padding: EdgeInsets.symmetric(horizontal: 16),
            //     child: BarChart(
            //       createChartData(hzValues, 'Hz'),
            //     ),
            //   ),
            // ),
            
            // SizedBox(height: 24),
            
            // // Latency Chart
            // AspectRatio(
            //   aspectRatio: 2,
            //   child: Padding(
            //     padding: EdgeInsets.symmetric(horizontal: 16),
            //     child: BarChart(
            //       createChartData(latencyValues, 'Latency (ms)'),
            //     ),
            //   ),
            // ),
            
            // SizedBox(height: 24),
            
            // // Bandwidth Chart
            // AspectRatio(
            //   aspectRatio: 2,
            //   child: Padding(
            //     padding: EdgeInsets.symmetric(horizontal: 16),
            //     child: BarChart(
            //       createChartData(bandwidthValues, 'Bandwidth (MB/s)'),
            //     ),
            //   ),
            // ),
            
            // SizedBox(height: 24),
            
            // Details Table
            Table(
              border: TableBorder.all(color: Colors.grey.shade300),
              children: [
                TableRow(
                  decoration: BoxDecoration(color: Colors.grey.shade100),
                  children: [
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text('Topic', style: TextStyle(fontWeight: FontWeight.bold)),
                    )),
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text('Hz', style: TextStyle(fontWeight: FontWeight.bold)),
                    )),
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text('Latency (ms)', style: TextStyle(fontWeight: FontWeight.bold)),
                    )),
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text('Bandwidth (MB/s)', style: TextStyle(fontWeight: FontWeight.bold)),
                    )),
                  ],
                ),
                ...topicStats.entries.map((entry) => TableRow(
                  children: [
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text(entry.key.split('/').last),
                    )),
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text(entry.value.hz.toStringAsFixed(1)),
                    )),
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text(entry.value.latency.toStringAsFixed(1)),
                    )),
                    TableCell(child: Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text(entry.value.bandwidth.toStringAsFixed(2)),
                    )),
                  ],
                )).toList(),
              ],
            ),
          ],
        ),
      ),
    );
  }

  @override
  void initState() {
    super.initState();
    _initializeTopics();
    _updateTimer = Timer.periodic(Duration(milliseconds: 50), (timer) {
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
              topicStats[topic]?.onMessageReceived(data.toString().length);
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
}