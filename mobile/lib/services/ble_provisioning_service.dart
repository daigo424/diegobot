import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

final _serviceUuid = Guid('00000001-710e-4a5b-8804-000000000000');
final _wifiConfigCharUuid = Guid('00000002-710e-4a5b-8804-000000000000');
final _statusCharUuid = Guid('00000003-710e-4a5b-8804-000000000000');

enum ProvisioningStatus { idle, connecting, connectingWifi, connected, failed }

@immutable
class ProvisioningState {
  final ProvisioningStatus status;
  final String? ip;
  final String? error;

  const ProvisioningState._(this.status, {this.ip, this.error});

  const ProvisioningState.idle() : this._(ProvisioningStatus.idle);
  const ProvisioningState.connecting() : this._(ProvisioningStatus.connecting);
  const ProvisioningState.connectingWifi() : this._(ProvisioningStatus.connectingWifi);
  const ProvisioningState.connected(String ip) : this._(ProvisioningStatus.connected, ip: ip);
  const ProvisioningState.failed(String error) : this._(ProvisioningStatus.failed, error: error);
}

class BleProvisioningService {
  final ValueNotifier<List<ScanResult>> devices = ValueNotifier(const []);
  final ValueNotifier<ProvisioningState> state = ValueNotifier(const ProvisioningState.idle());

  StreamSubscription<List<ScanResult>>? _scanSub;
  StreamSubscription<List<int>>? _statusSub;
  BluetoothDevice? _device;

  Future<void> startScan() async {
    devices.value = const [];
    await _scanSub?.cancel();
    _scanSub = FlutterBluePlus.scanResults.listen((results) {
      devices.value = results.where((r) => r.device.advName.startsWith('AMR-Setup-')).toList();
    });
    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 15));
  }

  Future<void> stopScan() async {
    await FlutterBluePlus.stopScan();
    await _scanSub?.cancel();
    _scanSub = null;
  }

  Future<void> connectAndSendCredentials(
    BluetoothDevice device,
    String ssid,
    String password,
  ) async {
    state.value = const ProvisioningState.connecting();
    await stopScan();

    // 非商用(個人・研究)利用のためnonprofitライセンス区分を使用。
    await device.connect(license: License.nonprofit);
    _device = device;

    // Write特性がencrypt-write必須(平文JSONでパスワードを送るため)なので、
    // 書込み前にボンディングしておかないとBlueZ側がWriteを拒否する。
    await device.createBond();

    final services = await device.discoverServices();
    final service = services.firstWhere((s) => s.uuid == _serviceUuid);
    final wifiConfigChar = service.characteristics.firstWhere((c) => c.uuid == _wifiConfigCharUuid);
    final statusChar = service.characteristics.firstWhere((c) => c.uuid == _statusCharUuid);

    await statusChar.setNotifyValue(true);
    await _statusSub?.cancel();
    _statusSub = statusChar.lastValueStream.listen(_onStatusNotify);

    final payload = utf8.encode(jsonEncode({'ssid': ssid, 'password': password}));
    await wifiConfigChar.write(payload);
  }

  void _onStatusNotify(List<int> value) {
    if (value.isEmpty) return;
    final decoded = jsonDecode(utf8.decode(value)) as Map<String, dynamic>;
    state.value = switch (decoded['status']) {
      'connecting' => const ProvisioningState.connectingWifi(),
      'connected' => ProvisioningState.connected(decoded['ip'] as String),
      'failed' => ProvisioningState.failed(decoded['error'] as String),
      _ => state.value,
    };
  }

  Future<void> dispose() async {
    await _scanSub?.cancel();
    await _statusSub?.cancel();
    await _device?.disconnect();
  }
}
