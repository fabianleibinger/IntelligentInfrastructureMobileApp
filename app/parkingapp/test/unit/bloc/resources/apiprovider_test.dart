import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';

void main() {
  StandardVehicle _standard;
  ChargeableVehicle _chargeable;

  setUp(() {
    _standard = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true);
    _chargeable = _standard.toChargeableVehicle();
  });

  group('Correct park in bodies should be chosen for vehicle types', () {
    test('Correct park in body should be chosen for StandardVehicle', () {
      var actualBody = ApiProvider.chooseParkInBody(_standard);
      var correctBody = ApiProvider.parkInBodyStandardVehicle(_standard);

      expect(actualBody, correctBody);
    });

    test('Correct park in body should be chosen for ChargeableVehicle', () {
      var actualBody = ApiProvider.chooseParkInBody(_chargeable);
      var correctBody = ApiProvider.parkInBodyChargeableVehicle(_chargeable);

      expect(actualBody, correctBody);
    });
  });

  tearDown(() {
    _standard = null;
    _chargeable = null;
  });
}
