import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/util/parkmanager.dart';

void main() {
  StandardVehicle _vehicle;

  setUp(() {
    _vehicle = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, false);
  });

  group('Should return if vehicle needs to park in for all vehicle states', () {
    test('Should return if parked in vehicle needs to park in', () {
      _vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkIn(null, _vehicle);

      expect(parkIn, false);
    });

    test('Should return if parked out vehicle needs to park in', () {
      _vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkIn(null, _vehicle);

      expect(parkIn, true);
    });

    test('Should return if parking in vehicle needs to park in', () {
      _vehicle.parkingIn = true;
      _vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkIn(null, _vehicle);

      expect(parkIn, false);
    });

    test('Should return if park in cancelling vehicle needs to park in', () {
      _vehicle.parkingOut = true;
      _vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkIn(null, _vehicle);

      expect(parkIn, false);
    });

    test('Should return if parking out vehicle needs to park in', () {
      _vehicle.parkingOut = true;
      _vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkIn(null, _vehicle);

      expect(parkIn, false);
    });
  });

  group('Should return if vehicle needs to park out for all vehicle states',
      () {
    test('Should return if parked in vehicle needs to park out', () {
      _vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkOut(null, _vehicle);

      expect(parkIn, true);
    });

    test('Should return if parked out vehicle needs to park out', () {
      _vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkOut(null, _vehicle);

      expect(parkIn, false);
    });

    test('Should return if parking in vehicle needs to park out', () {
      _vehicle.parkingIn = true;
      _vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkOut(null, _vehicle);

      expect(parkIn, true);
    });

    test('Should return if park in cancelling vehicle needs to park out', () {
      _vehicle.parkingOut = true;
      _vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkOut(null, _vehicle);

      expect(parkIn, false);
    });

    test('Should return if parking out vehicle needs to park out', () {
      _vehicle.parkingOut = true;
      _vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkOut(null, _vehicle);

      expect(parkIn, false);
    });
  });

  tearDown(() {
    _vehicle = null;
  });
}
