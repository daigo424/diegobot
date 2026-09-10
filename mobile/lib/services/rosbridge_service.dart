import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter_multicast_lock/flutter_multicast_lock.dart';
import 'package:multicast_dns/multicast_dns.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

const _robotHostname = 'diegobot.local';
const _reconnectDelay = Duration(seconds: 2);

class RosbridgeService {
  final ValueNotifier<bool> isConnected = ValueNotifier(false);

  final _multicastLock = FlutterMulticastLock();
  WebSocketChannel? _channel;
  Timer? _reconnectTimer;
  bool _stopped = true;

  Future<void> connect() async {
    _stopped = false;
    await _connectOnce();
  }

  // 自動リトライは2秒間隔だが、ユーザーが「今すぐ試したい」場合のために手動トリガーを用意する。
  // 保留中の自動リトライタイマーをキャンセルしてから即座に試行し、二重接続を防ぐ。
  Future<void> retryNow() async {
    if (_stopped) return;
    _reconnectTimer?.cancel();
    await _connectOnce();
  }

  Future<void> _connectOnce() async {
    if (_stopped) return;
    final ip = await _resolveRobotIp();
    if (ip == null) {
      _scheduleReconnect();
      return;
    }

    final channel = WebSocketChannel.connect(Uri.parse('ws://$ip:9090'));
    _channel = channel;
    try {
      await channel.ready;
    } catch (_) {
      _scheduleReconnect();
      return;
    }
    isConnected.value = true;
    channel.stream.listen(
      (_) {},
      onDone: _handleDisconnect,
      onError: (_) => _handleDisconnect(),
    );
  }

  // diegobot.localは固定ホスト名だが、AndroidはOSの通常のDNS解決経路が.localを
  // 素通りするため(iOS/macOSと違いBonjourがOSに統合されていない)、mDNSクエリを
  // 自前で投げて解決する必要がある。CHANGE_WIFI_MULTICAST_STATEでロックしないと
  // 端末によってはマルチキャスト応答パケットがWi-Fiスタックの省電力フィルタで
  // 破棄され、何も返ってこない。
  Future<String?> _resolveRobotIp() async {
    await _multicastLock.acquireMulticastLock();
    final client = MDnsClient();
    await client.start();
    try {
      final query = ResourceRecordQuery.addressIPv4(_robotHostname);
      await for (final record in client.lookup<IPAddressResourceRecord>(query)) {
        return record.address.address;
      }
      return null;
    } finally {
      client.stop();
      await _multicastLock.releaseMulticastLock();
    }
  }

  void _handleDisconnect() {
    isConnected.value = false;
    _scheduleReconnect();
  }

  void _scheduleReconnect() {
    if (_stopped) return;
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(_reconnectDelay, _connectOnce);
  }

  void publishCmdVel(double linearX, double angularZ) {
    if (!isConnected.value) return;
    final message = jsonEncode({
      'op': 'publish',
      'topic': '/cmd_vel',
      'type': 'geometry_msgs/msg/Twist',
      'msg': {
        'linear': {'x': linearX, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angularZ},
      },
    });
    _channel?.sink.add(message);
  }

  Future<void> stop() async {
    _stopped = true;
    _reconnectTimer?.cancel();
    isConnected.value = false;
    await _channel?.sink.close();
    _channel = null;
  }
}
