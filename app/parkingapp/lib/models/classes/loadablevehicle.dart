import 'package:parkingapp/models/classes/vehicle.dart';

class LoadableVehicle extends Vehicle {
  bool doCharge;
  String chargingProvider;
  List<DateTime> chargeTime;
  String charge;

  LoadableVehicle(
      key,
      name,
      licensePlate,
      height,
      width,
      length,
      turningCycle,
      nearExitPreference,
      parkingCard,
      this.doCharge,
      this.chargingProvider,
      this.chargeTime,
      this.charge)
      : super(key, name, licensePlate, height, width, length, turningCycle,
            nearExitPreference, parkingCard);

  static LoadableVehicle fromJson(Map<String, dynamic> parsedJson) {
    List<DateTime> chargetime = [
      parsedJson['chargetimebegin'],
      parsedJson['chargetimeend']
    ];

    return new LoadableVehicle(
        parsedJson['key'],
        parsedJson['name'],
        parsedJson['licenseplate'],
        int.parse(parsedJson['height']),
        int.parse(parsedJson['width']),
        int.parse(parsedJson['length']),
        int.parse(parsedJson['turningCycle']),
        parsedJson['nearExitPreference'].parseBool(),
        parsedJson['parkingCard'].parseBool(),
        parsedJson['doCharge'].parseBool(),
        parsedJson['chargingProvider'],
        chargetime,
        parsedJson['charge']);
  }

  //Todo
  String getBatteryCharge() {}
}
