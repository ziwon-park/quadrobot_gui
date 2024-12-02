import 'package:web_socket_channel/io.dart';
import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';

class RosService {
  IOWebSocketChannel? channel;
  String uri;

  RosService(this.uri);

  Future<void> connect() async {
    channel = await IOWebSocketChannel.connect(uri);
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

  void disconnect() {
    if (channel != null) {
      channel?.sink.close();
      channel = null;
    }
  }

}
