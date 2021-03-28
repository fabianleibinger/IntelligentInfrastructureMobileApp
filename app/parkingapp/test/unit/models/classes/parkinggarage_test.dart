import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

void main() {
  ParkingGarage garage;

  StandardVehicle standard;
  ChargeableVehicle chargeable;

  setUp(() {
    Coordinate coordinate = Coordinate(latitude: 0, longitude: 0);
    garage = ParkingGarage(
        name: '',
        type: ParkingGarageType.Tiefgarage,
        freeParkingSpots: 0,
        freeChargeableParkingSpots: 1,
        image: '',
        map: '',
        bottomLeft: coordinate,
        topRight: coordinate);

    standard = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true);
    chargeable = standard.toChargeableVehicle();
  });

  group('Free parking spots should be returned for vehicle types and settings',
      () {
    test('Free parking spots should be returned for StandardVehicle', () {
      int freeSpots = garage.getFreeSpotsForVehicle(standard);
      int correctSpots = garage.freeParkingSpots;

      expect(freeSpots, correctSpots);
    });

    test(
        'Free parking spots should be returned for ChargeableVehicle that enabled charging',
        () {
      chargeable.doCharge = true;

      int freeSpots = garage.getFreeSpotsForVehicle(chargeable);
      int correctSpots = garage.freeChargeableParkingSpots;

      expect(freeSpots, correctSpots);
    });

    test(
        'Free parking spots should be returned for ChargeableVehicle that disabled charging',
        () {
      chargeable.doCharge = false;

      int freeSpots = garage.getFreeSpotsForVehicle(chargeable);
      int correctSpots = garage.freeParkingSpots;

      expect(freeSpots, correctSpots);
    });
  });

  group(
      'Should return if specific parking spots are available for different numbers',
      () {
    test(
        'Should return if specific parking spots are available for 0 free parking spots',
        () {
      garage.freeParkingSpots = 0;

      bool available = garage.vehicleSpecificSpotsAvailable(standard);

      expect(available, false);
    });

    test(
        'Should return if specific parking spots are available for negative free parking spots',
        () {
      garage.freeParkingSpots = -1;

      bool available = garage.vehicleSpecificSpotsAvailable(standard);

      expect(available, false);
    });

    test(
        'Should return if specific parking spots are available for positive free parking spots',
        () {
      garage.freeParkingSpots = 1;

      bool available = garage.vehicleSpecificSpotsAvailable(standard);

      expect(available, true);
    });

    test(
        'Should return if specific parking spots are available for nAn free parking spots',
        () {
      garage.freeParkingSpots = null;

      expect(() => garage.vehicleSpecificSpotsAvailable(standard),
          throwsNoSuchMethodError);
    });
  });

  tearDown(() {
    garage = null;

    standard = null;
    chargeable = null;
  });
}
