import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';

void main() {
  ChargeableVehicle _chargeable;

  setUp(() {
    TimeOfDay time = TimeOfDay(hour: 0, minute: 0);
    _chargeable = ChargeableVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true, true, '', time , time);
  });

  test('To map and from map should result in the same vehicle values', () {
    var map = _chargeable.toMap();
    ChargeableVehicle fromMap = ChargeableVehicle.fromMap(map);

    expect(_chargeable.equals(fromMap), true);
  });

  test('To chargeable should result in the same vehicle values', () {
    var standard = _chargeable.toStandardVehicle();
    
    expect(_chargeable.equals(standard), true);
  });

  tearDown(() {
    _chargeable = null;
  });
}