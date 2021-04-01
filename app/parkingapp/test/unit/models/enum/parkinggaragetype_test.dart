import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

void main() {
  test('Value should be returned as short string', () {
    String value = ParkingGarageType.Parkgarage.toShortString();

    expect(value, 'Parkgarage');
  });
}