import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/main.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

void main() {
  group('MainPage has specific AppBar, AppDrawer, text, button', () {
    testWidgets('AppBar', (WidgetTester tester) async {
      await tester.pumpWidget(Main.getMaterialApp('AppDrawer()'));

      await tester.pump();

      expect(find.text('Messages'), findsOneWidget);
    });
  });
}
