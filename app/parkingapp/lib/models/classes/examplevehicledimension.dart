class ExampleVehicleDimension {
  String name;

  ///The vehicle dimensions.
  double height, width, length, turningCycle, distRearAxleLicensePlate;

  ExampleVehicleDimension(this.name, this.height, this.width, this.length,
      this.turningCycle, this.distRearAxleLicensePlate);

  static ExampleVehicleDimension fromJson(Map<String, dynamic> parsedJson) {
    return new ExampleVehicleDimension(
        parsedJson['name'],
        parsedJson['height'],
        parsedJson['width'],
        parsedJson['length'],
        parsedJson['turningCycle'],
        parsedJson['distRearAxleLicensePlate']);
  }
}
