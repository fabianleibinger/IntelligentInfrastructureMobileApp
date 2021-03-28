import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';

void main() {
  setUp(() {
    StandardVehicle standard =
        StandardVehicle('', '', '', 0, 0, 0, 0, 0, true, true, true);
    ChargeableVehicle chargeable = standard.toChargeableVehicle();
  });
}
