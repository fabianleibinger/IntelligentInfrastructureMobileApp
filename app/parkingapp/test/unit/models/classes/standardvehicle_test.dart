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

    expect(_standard.equals(fromMap), true);
  });

  test('To chargeable should result in the same vehicle values', () {
    var chargeable = _standard.toChargeableVehicle();

    expect(_standard.equals(chargeable), true);
  });

  tearDown(() {
    _standard = null;
  });
}