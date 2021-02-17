import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:parkingapp/main.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';

void main() {
  group('MainPage has specific AppBar, AppDrawer, text, button', () {
    testWidgets('AppBar', (WidgetTester tester) async {
      var appBar = find.byType(AppBar);
      //TODO replace hardcoded text
      var appBarText = find.text('Vehicle');

      await tester.pumpWidget(Main.getMaterialApp(MainPage()));

      expect(appBar, findsOneWidget);
      expect(appBarText, findsOneWidget);
    });

    testWidgets('AppDrawer', (WidgetTester tester) async {
      var appDrawer = find.byType(AppDrawer);

      await tester.pumpWidget(Main.getMaterialApp(AppDrawer()));

      expect(appDrawer, findsOneWidget);
      //TODO check text
    });

    testWidgets('parkingGarage information', (WidgetTester tester) async {
      var parkingGarageName = find.text(currentParkingGarage.name);
      var parkingGarageType =
          find.text(currentParkingGarage.type.toShortString());
      //TODO replace hardcoded text
      var freeParkingSpotsText = find.textContaining('Freie Parkplätze:');
      var freeParkingSpotsNumber =
          find.textContaining(currentParkingGarage.freeParkingSpots.toString());

      await tester.pumpWidget(Main.getMaterialApp(MainPage()));

      expect(parkingGarageName, findsOneWidget);
      expect(parkingGarageType, findsOneWidget);
      expect(freeParkingSpotsText, findsOneWidget);
      expect(freeParkingSpotsNumber, findsOneWidget);
    });

    testWidgets('preference information and some settings',
        (WidgetTester tester) async {
      await tester.pumpWidget(Main.getMaterialApp(MainPage()));

      //TODO replace text
      expect(find.textContaining('Fahrzeugpräferenzen:'), findsOneWidget);
    });

    testWidgets('parkInButton', (WidgetTester tester) async {
      var parkInButton = find.byType(FloatingActionButton);

      await tester.pumpWidget(Main.getMaterialApp(MainPage()));

      expect(parkInButton, findsOneWidget);

      await tester.tap(parkInButton);
      await tester.pump();

      //TODO update onPressed
      expect(find.text('Messages'), findsOneWidget);
    });
  });
}
