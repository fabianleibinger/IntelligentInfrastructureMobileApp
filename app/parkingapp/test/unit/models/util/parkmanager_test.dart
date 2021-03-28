import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/util/parkmanager.dart';

void main() {
  StandardVehicle vehicle;

  setUp(() {
    vehicle = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, false);
  });

  group('Should return if vehicle needs to park in for all vehicle states', () {
    test('Should return if parked in vehicle needs to park in', () {
      vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkIn(vehicle);

      expect(parkIn, false);
    });

    test('Should return if parked out vehicle needs to park in', () {
      vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkIn(vehicle);

      expect(parkIn, true);
    });

    test('Should return if parking in vehicle needs to park in', () {
      vehicle.parkingIn = true;
      vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkIn(vehicle);

      expect(parkIn, false);
    });

    test('Should return if park in cancelling vehicle needs to park in', () {
      vehicle.parkingOut = true;
      vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkIn(vehicle);

      expect(parkIn, false);
    });

    test('Should return if parking out vehicle needs to park in', () {
      vehicle.parkingOut = true;
      vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkIn(vehicle);

      expect(parkIn, false);
    });
  });


  group('Should return if vehicle needs to park out for all vehicle states', () {
    test('Should return if parked in vehicle needs to park out', () {
      vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkOut(vehicle);

      expect(parkIn, true);
    });

    test('Should return if parked out vehicle needs to park out', () {
      vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkOut(vehicle);

      expect(parkIn, false);
    });

    test('Should return if parking in vehicle needs to park out', () {
      vehicle.parkingIn = true;
      vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkOut(vehicle);

      expect(parkIn, true);
    });

    test('Should return if park in cancelling vehicle needs to park out', () {
      vehicle.parkingOut = true;
      vehicle.parkedIn = false;

      bool parkIn = ParkManager.needsToParkOut(vehicle);

      expect(parkIn, false);
    });

    test('Should return if parking out vehicle needs to park out', () {
      vehicle.parkingOut = true;
      vehicle.parkedIn = true;

      bool parkIn = ParkManager.needsToParkOut(vehicle);

      expect(parkIn, false);
    });
  });

  tearDown(() {
    vehicle = null;
  });
}