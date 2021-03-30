import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';

void main() {
  test('Value should be returned as short string', () {
    String value = ChargingProvider.Automatisch.toShortString();

    expect(value, 'Automatisch');
  });
}