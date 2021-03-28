import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';

void main() {
  StandardVehicle standard;
  ChargeableVehicle chargeable;

  setUp(() {
    standard = StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true);
    chargeable = standard.toChargeableVehicle();
  });

  group('Correct park in bodies should be chosen', () {
    test('Correct park in body should be chosen for StandardVehicle', () {
      var actualBody = ApiProvider.chooseParkInBody(standard);
      var correctBody = ApiProvider.parkInBodyStandardVehicle(standard);

      expect(actualBody, correctBody);
    });

    test('Correct park in body should be chosen for ChargeableVehicle', () {
      var actualBody = ApiProvider.chooseParkInBody(chargeable);
      var correctBody = ApiProvider.parkInBodyChargeableVehicle(chargeable);

      expect(actualBody, correctBody);
    });
  });

  tearDown(() {
    standard = null;
    chargeable = null;
  });
}
