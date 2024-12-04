import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:yaml/yaml.dart';

import './pages/main_page.dart';
import 'globals.dart' as globals;

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Quadruped Robot',
      theme: ThemeData(
        primarySwatch: Colors.grey,
        primaryColor: Colors.blue,
        scaffoldBackgroundColor: Colors.white,
        colorScheme: ColorScheme.light(
          primary: Colors.grey,
          secondary: Colors.blue,
        ),
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: InitialPage(),
    );
  }
}


class InitialPage extends StatefulWidget {
  @override
  _InitialPageState createState() => _InitialPageState();
}

class _InitialPageState extends State<InitialPage> {
  dynamic configData;
  String selectedRobot = 'Unitree Go1';

  void onPressed(ipController) async {
    final yamlString = await rootBundle.loadString('lib/assets/config.yaml');
    final yamlDoc = loadYaml(yamlString);

    String yamlIpAddress = yamlDoc['ip_address'];
    globals.fastapi_port = yamlDoc['fastapi_port'];
    globals.ip_address =  yamlIpAddress;

    // Load topic configurations
    if (yamlDoc['topics'] != null) {
      final topics = yamlDoc['topics'] as YamlMap;
      globals.topics['camera'] = topics['camera'];
      globals.topics['cmd_vel'] = topics['cmd_vel'];
      globals.topics['point_cloud'] = topics['point_cloud'];
      globals.topics['twist'] = topics['twist'];
    }

    String fullAddress = "ws://$yamlIpAddress:${ipController.text}";

    globals.full_address = fullAddress;
    print("새로운 IP 주소가 저장되었습니다: ${globals.full_address}");

    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => MainPage()),
    );
  }

  @override
  Widget build(BuildContext context) {
    final screenHeight = MediaQuery.of(context).size.height;
    final containerHeight = screenHeight * 0.8;
    final TextEditingController ipController = TextEditingController();


    return GestureDetector(
      onTap: (){
        FocusScope.of(context).unfocus();
      },
      child: Scaffold(
        body: SingleChildScrollView(
          // physics: NeverScrollableScrollPhysics(),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Padding(
                padding: EdgeInsets.only(top: 100.0),
                child: Image.asset('/home/ubuntuurl/ziwon/gui_ws/src/ros_web_app/lib/assets/url_logo.png', height: 100),
              ),
              Padding(
                padding: EdgeInsets.only(top: 15.0),
                // child: Text(
                //   'for Robotic Experiments',
                //   style: TextStyle(
                //     fontSize: 15,
                //     fontWeight: FontWeight.bold,
                //     color: Colors.grey,
                //   ),
                // ),
              ),
              SizedBox(height: 10),
              Container(
                height: containerHeight,
                margin: EdgeInsets.all(0),
                padding: EdgeInsets.symmetric(horizontal: 40, vertical: 40),
                decoration: BoxDecoration(
                  color: Colors.grey[300],
                  borderRadius: BorderRadius.circular(50),
                ),
                child: Column(
                  children: [
                    Image.asset('/home/ubuntuurl/ziwon/gui_ws/src/ros_web_app/lib/assets/unitree_go1.png', height: 200),
                    SizedBox(height: 30),
                    DropdownButtonFormField<String>(
                      decoration: InputDecoration(
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(40.0),
                        ),
                        filled: true,
                        fillColor: Colors.white,
                      ),
                      value: selectedRobot,
                      onChanged: (String? newValue) {
                        setState(() {
                          selectedRobot = newValue!;
                        });
                      },
                      items: <String>['Unitree Go1', 'Robot B2', 'Robot C3']
                          .map<DropdownMenuItem<String>>((String value) {
                        return DropdownMenuItem<String>(
                          value: value,
                          child: Text(value),
                        );
                      }).toList(),
                    ),
                    SizedBox(height: 24),
                    TextField(
                      controller: ipController,
                      decoration: InputDecoration(
                        hintText: 'Enter IP Address',
                        filled: true,
                        fillColor: Colors.white,
                        border: OutlineInputBorder(
                          borderSide: BorderSide(color: Colors.blue, width: 5.0),
                          borderRadius: BorderRadius.circular(40.0),
                        ),
                      ),
                      keyboardType: TextInputType.number,
                    ),
                    SizedBox(height: 24),
                    ElevatedButton(
                      style: ButtonStyle(
                        backgroundColor: MaterialStateProperty.all<Color>(Color.fromARGB(255, 4, 68, 148)),
                        shape: MaterialStateProperty.all<RoundedRectangleBorder>(
                          RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(40.0),
                          ),
                        ),
                      ),
                      onPressed: () => onPressed(ipController),
                      child: Text('START',style: TextStyle(
                        fontWeight: FontWeight.bold,
                        fontSize: 20,
                        color: Colors.white,
                      ),),
                    ),
                    SizedBox(height: 24),
                    Container(
                      alignment: Alignment.bottomCenter,
                      margin: EdgeInsets.fromLTRB(0, 0, 0, 30),
                      child: Text('@ Created by Jiwon Park, Urban Robotics Lab @ KAIST',
                          style: TextStyle(fontSize: 11, color: Colors.black54)),
                    )
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
