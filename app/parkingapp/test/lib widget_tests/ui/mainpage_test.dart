import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/main.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/enum/parkinggaragetype.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';

void main() {
  group('MainPage has specific AppBar, AppDrawer, text, button', () {
    testWidgets('AppBar', (WidgetTester tester) async {
      await tester.pumpWidget(AppDrawer());
  });
});
}
