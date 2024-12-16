import 'package:flutter/material.dart';
import '../ros_service.dart';
import '../globals.dart' as globals;

class ButtonPanel extends StatefulWidget {
  @override
  _ButtonPanelState createState() => _ButtonPanelState();
}

class _ButtonPanelState extends State<ButtonPanel> with SingleTickerProviderStateMixin {
  late RosService _rosService;
  Map<String, bool> buttonStates = {
    'STOP': false,
    'TASK': false,
    'AUTO': false,
    'EMERGENCY': false,
  };
  
  Map<String, bool> hoverStates = {
    'STOP': false,
    'TASK': false,
    'AUTO': false,
    'EMERGENCY': false,
  };

  late AnimationController _animationController;
  late Map<String, Animation<double>> _animations;

  @override
  void initState() {
    super.initState();
    _rosService = RosService(globals.full_address);
    _connectToRos();

    _animationController = AnimationController(
      duration: const Duration(milliseconds: 200),
      vsync: this,
    );

    _animations = {
      'STOP': _createAnimation(),
      'TASK': _createAnimation(),
      'AUTO': _createAnimation(),
      'EMERGENCY': _createAnimation(),
    };
  }

  Animation<double> _createAnimation() {
    return Tween<double>(
      begin: 0.6,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _animationController,
      curve: Curves.easeInOut,
    ));
  }

  Future<void> _connectToRos() async {
    await _rosService.connect();
  }

  void _handleButtonPress(String buttonType) {
    setState(() {
      if (buttonType == 'EMERGENCY') {
        buttonStates[buttonType] = !buttonStates[buttonType]!;
      } else {
        buttonStates[buttonType] = true;
      }
    });

    _animationController.forward().then((_) {
      _animationController.reverse();
    });

    String serviceName = '';
    switch (buttonType) {
      case 'STOP':
        serviceName = '/robot/stop';
        break;
      case 'TASK':
        serviceName = '/robot/task';
        break;
      case 'AUTO':
        serviceName = '/robot/auto';
        break;
      case 'EMERGENCY':
        serviceName = '/robot/emergency';
        break;
    }
    
    _rosService.callButtonService(serviceName);

    if (buttonType != 'EMERGENCY') {
      Future.delayed(Duration(milliseconds: 200), () {
        setState(() {
          buttonStates[buttonType] = false;
        });
      });
    }
  }

  Color _getButtonColor(String buttonType, bool isHovered, Animation<double> animation) {
    Color baseColor;
    switch (buttonType) {
      case 'STOP':
        baseColor = Color(0xFFE53935);
        break;
      case 'TASK':
        baseColor = Color(0xFF1E88E5); 
        break;
      case 'AUTO':
        baseColor = Color(0xFF43A047); 
        break;
      case 'EMERGENCY':
        baseColor = Color(0xFFFB8C00);  
        break;
      default:
        baseColor = Colors.grey;
    }

    if (buttonStates[buttonType]!) {
      return baseColor;
    }

    return baseColor.withOpacity(isHovered ? 0.8 : 0.6);
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text(
              'Control Buttons',
              style: TextStyle(
                fontSize: 20,
                // fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildAnimatedButton('STOP'),
                ),
                SizedBox(width: 8),
                Expanded(
                  child: _buildAnimatedButton('TASK'),
                ),
              ],
            ),
            SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  child: _buildAnimatedButton('AUTO'),
                ),
                SizedBox(width: 8),
                Expanded(
                  child: _buildAnimatedButton('EMERGENCY'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

    Widget _buildAnimatedButton(String buttonType) {
    return AnimatedBuilder(
        animation: _animations[buttonType]!,
        builder: (context, child) {
        return MouseRegion(
            onEnter: (_) => setState(() => hoverStates[buttonType] = true),
            onExit: (_) => setState(() => hoverStates[buttonType] = false),
            child: ElevatedButton(
            onPressed: () => _handleButtonPress(buttonType),
            style: ButtonStyle(
                backgroundColor: MaterialStateProperty.resolveWith<Color>((states) {
                return _getButtonColor(buttonType, hoverStates[buttonType]!, _animations[buttonType]!);
                }),
                foregroundColor: MaterialStateProperty.all(Colors.white),
                padding: MaterialStateProperty.all(EdgeInsets.symmetric(vertical: 16)),
                elevation: MaterialStateProperty.resolveWith<double>((states) {
                return hoverStates[buttonType]! ? 8 : 4;
                }),
                overlayColor: MaterialStateProperty.all(Colors.transparent),
                shadowColor: MaterialStateProperty.all(Colors.black.withOpacity(0.3)),
            ),
            child: Text(
                buttonType,
                style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
                shadows: [
                    Shadow(
                    offset: Offset(1, 1),
                    blurRadius: 2,
                    color: Colors.black.withOpacity(0.2),
                    ),
                ],
                ),
            ),
            ),
        );
        },
    );
    }


  @override
  void dispose() {
    _animationController.dispose();
    _rosService.disconnect();
    super.dispose();
  }
}