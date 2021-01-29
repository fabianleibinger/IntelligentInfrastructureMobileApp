import 'package:parkingapp/models/classes/vehicle.dart';

class StandardVehicle implements Vehicle {
  StandardVehicle(
      this.key,
      this.name,
      this.licensePlate,
      this.height,
      this.width,
      this.length,
      this.turningCycle,
      this.nearExitPreference,
      this.parkingCard);

  static StandardVehicle fromJson(Map<String, dynamic> parsedJson) {
    return new StandardVehicle(
        parsedJson['key'],
        parsedJson['name'],
        parsedJson['licensePlate'],
        int.parse(parsedJson['height']),
        int.parse(parsedJson['width']),
        int.parse(parsedJson['length']),
        int.parse(parsedJson['turningCycle']),
        parsedJson['nearExitPreference'].parseBool(),
        parsedJson['parkingCard'].parseBool());
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
