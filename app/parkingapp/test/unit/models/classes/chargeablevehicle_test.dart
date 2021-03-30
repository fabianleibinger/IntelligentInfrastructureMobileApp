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

    expect(fromMap.inAppKey, _chargeable.inAppKey);
    expect(fromMap.databaseId, _chargeable.databaseId);
    expect(fromMap.name, _chargeable.name);
    expect(fromMap.licensePlate, _chargeable.licensePlate);
    expect(fromMap.width, _chargeable.width);
    expect(fromMap.height, _chargeable.height);
    expect(fromMap.length, _chargeable.length);
    expect(fromMap.turningCycle, _chargeable.turningCycle);
    expect(fromMap.distRearAxleLicensePlate, _chargeable.distRearAxleLicensePlate);
    expect(fromMap.nearExitPreference, _chargeable.nearExitPreference);
    expect(fromMap.parkingCard, _chargeable.parkingCard);
    expect(fromMap.parkedIn, _chargeable.parkedIn);
    expect(fromMap.parkingIn, _chargeable.parkingIn);
    expect(fromMap.parkingOut, _chargeable.parkingOut);
    expect(fromMap.doCharge, _chargeable.doCharge);
    expect(fromMap.chargingProvider, _chargeable.chargingProvider);
    expect(fromMap.chargeTimeBegin, _chargeable.chargeTimeBegin);
    expect(fromMap.chargeTimeEnd, _chargeable.chargeTimeEnd);
  });

  test('To chargeable should result in the same vehicle values', () {
    var standard = _chargeable.toStandardVehicle();
    
    expect(_chargeable.equals(standard), true);
  });

  tearDown(() {
    _chargeable = null;
  });
}