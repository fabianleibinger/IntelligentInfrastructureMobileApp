import 'package:parkingapp/models/classes/vehicle.dart';

class LoadableVehicle implements Vehicle {
  bool doCharge;
  String chargingProvider;
  List<DateTime> chargeTime;
  String charge;

  LoadableVehicle(
      this.key,
      this.name,
      this.licensePlate,
      this.height,
      this.width,
      this.length,
      this.turningCycle,
      this.nearExitPreference,
      this.parkingCard,
      this.doCharge,
      this.chargingProvider,
      this.chargeTime,
      this.charge);

  static LoadableVehicle fromJson(Map<String, dynamic> parsedJson) {
    List<DateTime> chargeTime = [
      parsedJson['chargeTimeBegin'],
      parsedJson['chargeTimeEnd']
    ];

    return new LoadableVehicle(
        parsedJson['key'],
        parsedJson['name'],
        parsedJson['licensePlate'],
        int.parse(parsedJson['height']),
        int.parse(parsedJson['width']),
        int.parse(parsedJson['length']),
        int.parse(parsedJson['turningCycle']),
        parsedJson['nearExitPreference'].parseBool(),
        parsedJson['parkingCard'].parseBool(),
        parsedJson['doCharge'].parseBool(),
        parsedJson['chargingProvider'],
        chargeTime,
        parsedJson['charge']);
  }

  String getBatteryCharge() {
    // TODO: implement getBatteryCharge
    throw UnimplementedError();
  }

  @override
  String setDimensions() {
    // TODO: implement setDimensions
    throw UnimplementedError();
  }

  @override
  String setPreferences() {
    // TODO: implement setPreferences
    throw UnimplementedError();
  }

  @override
  String key, name, licensePlate;

  @override
  int height, width, length, turningCycle;

  @override
  bool nearExitPreference, parkingCard;
}
