import 'package:web_socket_channel/web_socket_channel.dart';
import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';

class RosService {
  WebSocketChannel? channel;
  String uri;

  RosService(this.uri);

  Future<void> connect() async {
    try {
      channel = WebSocketChannel.connect(Uri.parse(uri));
      // 연결 후 상태 확인을 위한 ping 메시지 전송
      var pingMessage = {
        'op': 'ping'
      };
      channel!.sink.add(json.encode(pingMessage));
    } catch (e) {
      throw Exception('Failed to connect to ROS: $e');
    }
  }

  bool isConnected() {
    return channel != null;
  }

  void publish(String message) {
    if (channel != null) {
      channel!.sink.add(message);
    }
  }



  void subscribeImage(String topic, Function(Uint8List data, int dataSize) callback) {
    var message = {
      'op': 'subscribe',
      'topic': topic,
      // 'type': 'sensor_msgs/Image',
      'type': 'sensor_msgs/CompressedImage',
    };
    channel?.sink.add(json.encode(message));
    channel?.stream.listen((message) {
      var data = json.decode(message);
      if (data['msg'] != null && data['msg']['data'] != null) {
        // Base64 문자열을 디코딩하여 Uint8List로 변환
        Uint8List imageData = base64.decode(data['msg']['data']);
        int dataSize = imageData.length;
        callback(imageData, dataSize);
      }
    });
  }

  void subscribeToPointCloud(String topic, Function(List<double>) callback) {
    var message = {
      'op': 'subscribe',
      'topic': topic,
      'type': 'sensor_msgs/PointCloud2',
    };
    channel?.sink.add(json.encode(message));
    
    channel?.stream.listen((message) {
      var data = json.decode(message);
      if (data['msg'] != null && data['msg']['data'] != null) {
        List<int> rawData = base64.decode(data['msg']['data']);
        List<double> points = [];
        
        // PointCloud2 데이터를 float32 배열로 변환
        for (int i = 0; i < rawData.length; i += 4) {
          if (i + 3 < rawData.length) {
            var bytes = Uint8List.fromList(rawData.sublist(i, i + 4));
            var value = ByteData.sublistView(bytes).getFloat32(0, Endian.little);
            points.add(value);
          }
        }
        
        callback(points);
      }
    });
  }

  void subscribeToMap(String topic, Function(List<int> data, int width, int height, double resolution) callback) {
    var subscribeMessage = {
      'op': 'subscribe',
      'topic': topic,
      'type': 'nav_msgs/OccupancyGrid',
    };
    channel?.sink.add(json.encode(subscribeMessage));
    
    // 명시적으로 한번 요청
    var requestMessage = {
      'op': 'call_service',
      'service': '/static_map',  // static_map 서비스 사용
      'type': 'nav_msgs/GetMap',
      'args': {}
    };
    channel?.sink.add(json.encode(requestMessage));
    
    channel?.stream.listen((message) {
      var data = json.decode(message);
      if (data['msg'] != null) {
        var msg = data['msg'];
        var info = msg['info'];
        var width = info['width'];
        var height = info['height'];
        var resolution = info['resolution'].toDouble();
        
        // Convert base64 data to List<int>
        List<int> mapData = List<int>.from(msg['data']);
        
        callback(mapData, width, height, resolution);
      }
    });
  }

  void subscribeTopic(String topic, Function(dynamic) callback) {
    var message = {
      'op': 'subscribe',
      'topic': topic,
    };
    channel?.sink.add(json.encode(message));
    channel?.stream.listen((message) {
      var data = json.decode(message);
      if (data['msg'] != null) {
        callback(data['msg']);
      }
    });
  }

  void disconnect() {
    if (channel != null) {
      channel?.sink.close();
      channel = null;
    }
  }

}
