import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

import '../services/rosbridge_service.dart';

const _maxLinearSpeed = 0.5; // m/s
const _maxAngularSpeed = 1.0; // rad/s
const _sendPeriod = Duration(milliseconds: 40); // 25Hz (仕様の20〜30Hz範囲内)

class JoystickScreen extends StatefulWidget {
  const JoystickScreen({super.key});

  @override
  State<JoystickScreen> createState() => _JoystickScreenState();
}

class _JoystickScreenState extends State<JoystickScreen> {
  final _rosbridge = RosbridgeService();

  @override
  void initState() {
    super.initState();
    _rosbridge.connect();
  }

  @override
  void dispose() {
    _rosbridge.stop();
    super.dispose();
  }

  void _onDrag(StickDragDetails details) {
    _rosbridge.publishCmdVel(-details.y * _maxLinearSpeed, -details.x * _maxAngularSpeed);
  }

  void _onDragEnd() {
    // 指を離した瞬間にゼロ速度を即送信しないと、直前の速度指令のままモーターが回り続ける。
    _rosbridge.publishCmdVel(0, 0);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('操縦 (diegobot.local)')),
      body: Column(
        children: [
          ValueListenableBuilder<bool>(
            valueListenable: _rosbridge.isConnected,
            builder: (context, connected, _) {
              if (connected) return const SizedBox.shrink();
              return Container(
                width: double.infinity,
                color: Colors.red,
                padding: const EdgeInsets.all(8),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const Text(
                      '未接続',
                      style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold),
                    ),
                    const SizedBox(width: 12),
                    TextButton(
                      onPressed: _rosbridge.retryNow,
                      style: TextButton.styleFrom(foregroundColor: Colors.white),
                      child: const Text('再接続'),
                    ),
                  ],
                ),
              );
            },
          ),
          Expanded(
            child: Center(
              child: Joystick(
                period: _sendPeriod,
                listener: _onDrag,
                onStickDragEnd: _onDragEnd,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
