import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';

void main() {
  StandardVehicle _standard;

  setUp(() {
    _standard = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true);
  });

  test('To map and from map should result in the same vehicle values', () {
    var map = _standard.toMap();
    StandardVehicle fromMap = StandardVehicle.fromMap(map);

    expect(fromMap.inAppKey, _standard.inAppKey);
    expect(fromMap.databaseId, _standard.databaseId);
    expect(fromMap.name, _standard.name);
    expect(fromMap.licensePlate, _standard.licensePlate);
    expect(fromMap.width, _standard.width);
    expect(fromMap.height, _standard.height);
    expect(fromMap.length, _standard.length);
    expect(fromMap.turningCycle, _standard.turningCycle);
    expect(fromMap.distRearAxleLicensePlate, _standard.distRearAxleLicensePlate);
    expect(fromMap.nearExitPreference, _standard.nearExitPreference);
    expect(fromMap.parkingCard, _standard.parkingCard);
    expect(fromMap.parkedIn, _standard.parkedIn);
    expect(fromMap.parkingIn, _standard.parkingIn);
    expect(fromMap.parkingOut, _standard.parkingOut);
  });

  test('To chargeable should result in the same vehicle values', () {
    var chargeable = _standard.toChargeableVehicle();

    expect(_standard.equals(chargeable), true);
  });

  tearDown(() {
    _standard = null;
  });
}