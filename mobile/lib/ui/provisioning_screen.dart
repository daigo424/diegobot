import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

import '../services/ble_provisioning_service.dart';
import 'joystick_screen.dart';

class ProvisioningScreen extends StatefulWidget {
  const ProvisioningScreen({super.key});

  @override
  State<ProvisioningScreen> createState() => _ProvisioningScreenState();
}

class _ProvisioningScreenState extends State<ProvisioningScreen> {
  final _service = BleProvisioningService();
  final _ssidController = TextEditingController();
  final _passwordController = TextEditingController();
  BluetoothDevice? _selectedDevice;
  bool _navigated = false;

  @override
  void initState() {
    super.initState();
    _service.state.addListener(_onStateChanged);
    _requestPermissionsAndScan();
  }

  Future<void> _requestPermissionsAndScan() async {
    await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.locationWhenInUse,
    ].request();
    await _service.startScan();
  }

  void _onStateChanged() {
    setState(() {});
    final state = _service.state.value;
    if (state.status == ProvisioningStatus.connected && !_navigated) {
      _navigated = true;
      Navigator.of(context).pushReplacement(
        MaterialPageRoute(builder: (_) => const JoystickScreen()),
      );
    }
  }

  @override
  void dispose() {
    _service.state.removeListener(_onStateChanged);
    _service.dispose();
    _ssidController.dispose();
    _passwordController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Wi-Fi設定')),
      body: _selectedDevice == null ? _buildDeviceList() : _buildCredentialsForm(),
    );
  }

  Widget _buildDeviceList() {
    return ValueListenableBuilder<List<ScanResult>>(
      valueListenable: _service.devices,
      builder: (context, results, _) {
        if (results.isEmpty) {
          return Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const CircularProgressIndicator(),
                const SizedBox(height: 16),
                const Text('ロボットを探しています...'),
                const SizedBox(height: 24),
                // ロボットが既にWi-Fi接続済みだとBLEアドバタイズが休止されスキャンに出てこないため、
                // その場合に備えてBLEプロビジョニングを経由せず直接ジョイスティック画面へ進めるようにする。
                TextButton(
                  onPressed: () => Navigator.of(context).pushReplacement(
                    MaterialPageRoute(builder: (_) => const JoystickScreen()),
                  ),
                  child: const Text('スキップ (ロボットは既にWi-Fi接続済み)'),
                ),
              ],
            ),
          );
        }
        return ListView.builder(
          itemCount: results.length,
          itemBuilder: (context, index) {
            final device = results[index].device;
            return ListTile(
              leading: const Icon(Icons.bluetooth),
              title: Text(device.advName),
              onTap: () => setState(() => _selectedDevice = device),
            );
          },
        );
      },
    );
  }

  Widget _buildCredentialsForm() {
    final state = _service.state.value;
    return Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Text('接続先: ${_selectedDevice!.advName}'),
          const SizedBox(height: 16),
          TextField(
            controller: _ssidController,
            decoration: const InputDecoration(labelText: 'SSID'),
          ),
          const SizedBox(height: 8),
          TextField(
            controller: _passwordController,
            decoration: const InputDecoration(labelText: 'パスワード'),
            obscureText: true,
          ),
          const SizedBox(height: 16),
          FilledButton(
            onPressed: state.status == ProvisioningStatus.connecting ||
                    state.status == ProvisioningStatus.connectingWifi
                ? null
                : () => _service.connectAndSendCredentials(
                      _selectedDevice!,
                      _ssidController.text,
                      _passwordController.text,
                    ),
            child: const Text('送信'),
          ),
          const SizedBox(height: 16),
          _buildStatusText(state),
        ],
      ),
    );
  }

  Widget _buildStatusText(ProvisioningState state) {
    final text = switch (state.status) {
      ProvisioningStatus.connecting => 'ロボットへ接続中...',
      ProvisioningStatus.connectingWifi => 'Wi-Fi接続中...',
      ProvisioningStatus.failed => '接続失敗: ${state.error}',
      ProvisioningStatus.connected => '接続成功: ${state.ip}',
      ProvisioningStatus.idle => '',
    };
    return Text(text, style: Theme.of(context).textTheme.bodyMedium);
  }
}
