import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/routes/routes.dart';

void main() {
  Vehicle vehicle;

  setUp(() {
    vehicle = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, false);
  });

  group('Should return correct route for all vehicle states', () {
    test('Should return correct route for parked in vehicle', () {
      vehicle.parkedIn = true;

      String route = Routes.returnCorrectRouteForVehicle(vehicle);
      String correctRoute = vehicle.inAppKey + Routes.parkIn;
      expect(route, correctRoute);
    });

    test('Should return correct route for parked out vehicle', () {
      vehicle.parkedIn = false;

      String route = Routes.returnCorrectRouteForVehicle(vehicle);
      String correctRoute = vehicle.inAppKey;
      expect(route, correctRoute);
    });

    test('Should return correct route for parking in vehicle', () {
      vehicle.parkingIn = true;
      vehicle.parkedIn = false;

      String route = Routes.returnCorrectRouteForVehicle(vehicle);
      String correctRoute = vehicle.inAppKey + Routes.parkIn;
      expect(route, correctRoute);
    });

    test('Should return correct route for park in cancelling vehicle', () {
      vehicle.parkingOut = true;
      vehicle.parkedIn = false;

      String route = Routes.returnCorrectRouteForVehicle(vehicle);
      String correctRoute = vehicle.inAppKey + Routes.parkOut;
      expect(route, correctRoute);
    });

    test('Should return correct route for parking out vehicle', () {
      vehicle.parkingOut = true;
      vehicle.parkedIn = true;

      String route = Routes.returnCorrectRouteForVehicle(vehicle);
      String correctRoute = vehicle.inAppKey + Routes.parkOut;
      expect(route, correctRoute);
    });
  });

  tearDown(() {
    vehicle = null;
  });
}