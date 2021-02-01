import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/enum/parkinggaragetype.dart';

void main() {
  testWidgets(
      'mainpage has a specific appBar including title, appDrawer, text and a button',
      (WidgetTester tester) async {
    tester.pumpWidget(MainPage());

    expect(find.byWidget(AppBar()), findsOneWidget);
    //TODO replace 'vehicle'  by correct title
    expect(find.text('vehicle'), findsOneWidget);

    expect(find.byWidget(AppDrawer()), findsOneWidget);

    expect(find.text(currentParkingGarage.name), findsOneWidget);
    expect(
        find.text(currentParkingGarage.type.toShortString()), findsOneWidget);
  });
}
