class Vehicle {
  String key, name, licenseplate;
  List<int> dimensions;
  List<bool> preferences;

  Vehicle(this.key, this.name, this.licenseplate, this.dimensions,
      this.preferences);

  static Vehicle fromJson(Map<String, dynamic> parsedJson) {
    List<int> dimensions = [
      int.parse(parsedJson['dimensionH']),
      int.parse(parsedJson['dimensionW']),
      int.parse(parsedJson['dimensionD'])
    ];
    List<bool> preferences = [
      parsedJson['nearExit'].parseBool(),
      parsedJson['parkingcard'].parseBool()
    ];
    return new Vehicle(parsedJson['key'], parsedJson['name'],
        parsedJson['licenseplate'], dimensions, preferences);
  }

  //Todo
  String toString() {}

  //Todo
  String setPreferences(List<bool> preferences) {}
}

class ElectricalVehicle extends Vehicle {
  bool docharge;
  String chargingprovider;
  List<DateTime> chargetime;
  String charge;

  ElectricalVehicle(key, name, licenseplate, dimensions, preferences,
      this.docharge, this.chargingprovider, this.chargetime, this.charge)
      : super(key, name, licenseplate, dimensions, preferences);

  static ElectricalVehicle fromJson(Map<String, dynamic> parsedJson) {
    List<int> dimensions = [
      int.parse(parsedJson['dimensionH']),
      int.parse(parsedJson['dimensionW']),
      int.parse(parsedJson['dimensionD'])
    ];
    List<bool> preferences = [
      parsedJson['nearExit'].parseBool(),
      parsedJson['parkingcard'].parseBool()
    ];
    List<DateTime> chargetime = [
      parsedJson['chargetimebegin'],
      parsedJson['chargetimeend']
    ];

    return new ElectricalVehicle(
        parsedJson['key'],
        parsedJson['name'],
        parsedJson['licenseplate'],
        dimensions,
        preferences,
        parsedJson['docharge'],
        parsedJson['chargingprovider'],
        chargetime,
        parsedJson['charge']);
  }

  //Todo
  String getBatteryCharge() {}
}
