class Vehicle {
  String key, name, licenseplate;
  double width, height, depth, turnangle;
  bool nearexit, parkingcard;

  Vehicle(this.key, this.name, this.licenseplate, this.width, this.height,
      this.depth, this.turnangle, this.nearexit, this.parkingcard);

  // static Vehicle fromJson(Map<String, dynamic> parsedJson) {
  //   List<int> dimensions = [
  //     int.parse(parsedJson['dimensionH']),
  //     int.parse(parsedJson['dimensionW']),
  //     int.parse(parsedJson['dimensionD'])
  //   ];
  //   List<bool> preferences = [
  //     parsedJson['nearExit'].parseBool(),
  //     parsedJson['parkingcard'].parseBool()
  //   ];
  //   return new Vehicle(parsedJson['key'], parsedJson['name'],
  //       parsedJson['licenseplate'], dimensions, preferences);
  // }

  //Todo
  String toString() {}

  //Todo
  String setPreferences(bool nearexit, bool parkingcard) {}
}

class ElectricalVehicle extends Vehicle {
  bool docharge;
  String chargingprovider;
  DateTime chargebegin, chargeend;

  ElectricalVehicle(
      key,
      name,
      licenseplate,
      width,
      height,
      depth,
      turnangle,
      nearexit,
      parkingcard,
      this.docharge,
      this.chargingprovider,
      this.chargebegin,
      this.chargeend)
      : super(key, name, licenseplate, width, height, depth, turnangle,
            nearexit, parkingcard);

  // static ElectricalVehicle fromJson(Map<String, dynamic> parsedJson) {
  //   List<int> dimensions = [
  //     int.parse(parsedJson['dimensionH']),
  //     int.parse(parsedJson['dimensionW']),
  //     int.parse(parsedJson['dimensionD'])
  //   ];
  //   List<bool> preferences = [
  //     parsedJson['nearExit'].parseBool(),
  //     parsedJson['parkingcard'].parseBool()
  //   ];
  //   List<DateTime> chargetime = [
  //     parsedJson['chargetimebegin'],
  //     parsedJson['chargetimeend']
  //   ];
  //
  //   return new ElectricalVehicle(
  //       parsedJson['key'],
  //       parsedJson['name'],
  //       parsedJson['licenseplate'],
  //       dimensions,
  //       preferences,
  //       parsedJson['docharge'],
  //       parsedJson['chargingprovider'],
  //       chargetime,
  //       parsedJson['charge']);
  // }

  //Todo
  //String getBatteryCharge() {}
}
