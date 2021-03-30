import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

void main() {
  ParkingGarage _garage;

  StandardVehicle _standard;
  ChargeableVehicle _chargeable;

  setUp(() {
    Coordinate coordinate = Coordinate(latitude: 0, longitude: 0);
    _garage = ParkingGarage(
        name: '',
        type: ParkingGarageType.Tiefgarage,
        freeParkingSpots: 0,
        freeChargeableParkingSpots: 1,
        image: '',
        map: '',
        bottomLeft: coordinate,
        topRight: coordinate);

    _standard = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true);
    _chargeable = _standard.toChargeableVehicle();
  });

  group('Free parking spots should be returned for vehicle types and settings',
      () {
    test('Free parking spots should be returned for StandardVehicle', () {
      int freeSpots = _garage.getFreeSpotsForVehicle(_standard);
      int correctSpots = _garage.freeParkingSpots;

      expect(freeSpots, correctSpots);
    });

    test(
        'Free parking spots should be returned for ChargeableVehicle that enabled charging',
        () {
      _chargeable.doCharge = true;

      int freeSpots = _garage.getFreeSpotsForVehicle(_chargeable);
      int correctSpots = _garage.freeChargeableParkingSpots;

      expect(freeSpots, correctSpots);
    });

    test(
        'Free parking spots should be returned for ChargeableVehicle that disabled charging',
        () {
      _chargeable.doCharge = false;

      int freeSpots = _garage.getFreeSpotsForVehicle(_chargeable);
      int correctSpots = _garage.freeParkingSpots;

      expect(freeSpots, correctSpots);
    });
  });

  group(
      'Should return if specific parking spots are available for different numbers',
      () {
    test(
        'Should return if specific parking spots are available for 0 free parking spots',
        () {
      _garage.freeParkingSpots = 0;

      bool available = _garage.vehicleSpecificSpotsAvailable(_standard);

      expect(available, false);
    });

    test(
        'Should return if specific parking spots are available for negative free parking spots',
        () {
      _garage.freeParkingSpots = -1;

      bool available = _garage.vehicleSpecificSpotsAvailable(_standard);

      expect(available, false);
    });

    test(
        'Should return if specific parking spots are available for positive free parking spots',
        () {
      _garage.freeParkingSpots = 1;

      bool available = _garage.vehicleSpecificSpotsAvailable(_standard);

      expect(available, true);
    });

    test(
        'Should return if specific parking spots are available for nAn free parking spots',
        () {
      _garage.freeParkingSpots = null;

      expect(() => _garage.vehicleSpecificSpotsAvailable(_standard),
          throwsNoSuchMethodError);
    });
  });

  tearDown(() {
    _garage = null;

    _standard = null;
    _chargeable = null;
  });
}
