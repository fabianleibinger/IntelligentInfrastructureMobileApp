import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/util/utility.dart';

void main() {
  group('test regular tiles', () {
    test('tests the highlighting of a tile. This tile should be selected', () {
      var tile = generateTile(null, 'drawer', 'drawer', 'Title', Icons.ac_unit);
      expect(tile.selected, true);
    });

    test('tests the highlighting of a tile. This tile should notbe selected',
        () {
      var tile =
          generateTile(null, 'currentDrawer', 'drawer', 'Title', Icons.ac_unit);
      expect(tile.selected, false);
    });
  });

  group('test vehicle tiles', () {
    String _name = 'vehicle', _licensePlate = 'licensplate';
    double _width = 10,
        _height = 10,
        _length = 10,
        _turningCycle = 10,
        _distRearAxleLicensePlate = 10;
    bool _nearExitPreference = false, _parkingCard = false, _parkedIn = false;
    StandardVehicle _standardVehicle;
    String _key;
    setUp(() {
      _key = Utility.generateKey();
      _standardVehicle = StandardVehicle(
          _key,
          _name,
          _licensePlate,
          _width,
          _height,
          _length,
          _turningCycle,
          _distRearAxleLicensePlate,
          _nearExitPreference,
          _parkingCard,
          _parkedIn);
    });

    test('test if vehicle tile is selected', () {
      var tile = generateVehicleTile(
          null, _key.toString(), _standardVehicle, Icons.directions_car);
      expect(tile.selected, true);
    });
    test('test if vehicle tile is not selected', () {
      var tile = generateVehicleTile(
          null, Utility.generateKey(), _standardVehicle, Icons.directions_car);
      expect(tile.selected, false);
    });

    testWidgets('test if the vehicle title matches the the actual title',
        (WidgetTester tester) async {
      await tester.pumpWidget(MaterialApp(
          home: Scaffold(
              body: generateVehicleTile(null, _key.toString(), _standardVehicle,
                  Icons.directions_car))));
      expect(find.text(_name), findsOneWidget);
    });
  });
}
