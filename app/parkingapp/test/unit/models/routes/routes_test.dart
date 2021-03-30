import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/routes/routes.dart';

void main() {
  Vehicle _vehicle;

  setUp(() {
    _vehicle = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, false);
  });

  group('Should return correct route for all vehicle states', () {
    test('Should return correct route for parked in vehicle', () {
      _vehicle.parkedIn = true;

      String route = Routes.returnCorrectRouteForVehicle(_vehicle);
      String correctRoute = _vehicle.inAppKey + Routes.parkIn;
      expect(route, correctRoute);
    });

    test('Should return correct route for parked out vehicle', () {
      _vehicle.parkedIn = false;

      String route = Routes.returnCorrectRouteForVehicle(_vehicle);
      String correctRoute = _vehicle.inAppKey;
      expect(route, correctRoute);
    });

    test('Should return correct route for parking in vehicle', () {
      _vehicle.parkingIn = true;
      _vehicle.parkedIn = false;

      String route = Routes.returnCorrectRouteForVehicle(_vehicle);
      String correctRoute = _vehicle.inAppKey + Routes.parkIn;
      expect(route, correctRoute);
    });

    test('Should return correct route for park in cancelling vehicle', () {
      _vehicle.parkingOut = true;
      _vehicle.parkedIn = false;

      String route = Routes.returnCorrectRouteForVehicle(_vehicle);
      String correctRoute = _vehicle.inAppKey + Routes.parkOut;
      expect(route, correctRoute);
    });

    test('Should return correct route for parking out vehicle', () {
      _vehicle.parkingOut = true;
      _vehicle.parkedIn = true;

      String route = Routes.returnCorrectRouteForVehicle(_vehicle);
      String correctRoute = _vehicle.inAppKey + Routes.parkOut;
      expect(route, correctRoute);
    });
  });

  tearDown(() {
    _vehicle = null;
  });
}