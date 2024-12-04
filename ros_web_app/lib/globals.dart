library globals;

String ip_address = "";
String full_address = "";
String rosbridge_port = "";
String fastapi_port = "";

// ROS Topics
Map<String, String> topics = {
  'camera': '',
  'cmd_vel': '',
  'point_cloud': '',
  'twist': '',
};