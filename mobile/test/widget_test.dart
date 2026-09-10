import 'package:flutter_test/flutter_test.dart';

import 'package:diegoctl/main.dart';

void main() {
  testWidgets('provisioning screen appears on launch', (WidgetTester tester) async {
    await tester.pumpWidget(const DiegoctlApp());
    expect(find.text('Wi-Fi設定'), findsOneWidget);
  });
}
