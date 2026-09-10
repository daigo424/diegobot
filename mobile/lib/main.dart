import 'package:flutter/material.dart';

import 'ui/provisioning_screen.dart';

void main() {
  runApp(const DiegoctlApp());
}

class DiegoctlApp extends StatelessWidget {
  const DiegoctlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'diegoctl',
      theme: ThemeData(colorSchemeSeed: Colors.blue, useMaterial3: true),
      home: const ProvisioningScreen(),
    );
  }
}
