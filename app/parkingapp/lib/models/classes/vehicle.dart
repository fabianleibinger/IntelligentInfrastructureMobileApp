class Vehicle {
  String key, name, licensePlate;
  //Dimensions
  int height, width, length, turningCycle;
  //Preferences
  bool nearExitPreference, parkingCard;

  Vehicle(
      this.key,
      this.name,
      this.licensePlate,
      this.height,
      this.width,
      this.length,
      this.turningCycle,
      this.nearExitPreference,
      this.parkingCard);

  static Vehicle fromJson(Map<String, dynamic> parsedJson) {
    return new Vehicle(
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

  //Todo
  String setDimensions() {}

  //Todo
  String setPreferences() {}

  //Todo
  String toString() {}
}
