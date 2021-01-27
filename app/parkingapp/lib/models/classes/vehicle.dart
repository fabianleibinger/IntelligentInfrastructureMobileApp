import 'package:parkingapp/models/data/vehicles.dart';

class Vehicle {
  int id;
  String key, name, licenseplate;
  double width, height, depth, turnangle;
  bool nearexit, parkingcard;

  Vehicle(this.id, this.key, this.name, this.licenseplate, this.width,
      this.height, this.depth, this.turnangle, this.nearexit, this.parkingcard);

  Map<String, dynamic> toMap() {
    var map = <String, dynamic>{
      DatabaseProvider.COLUMN_KEY: key,
      DatabaseProvider.COLUMN_NAME: name,
      DatabaseProvider.COLUMN_LICENSEPLATE: licenseplate,
      DatabaseProvider.COLUMN_WIDTH: width.toDouble(),
      DatabaseProvider.COLUMN_HEIGHT: height.toDouble(),
      DatabaseProvider.COLUMN_DEPTH: depth.toDouble(),
      DatabaseProvider.COLUMN_TURNANGLE: turnangle.toDouble(),
      DatabaseProvider.COLUMN_NEAREXIT: nearexit ? 1 : 0,
      DatabaseProvider.COLUMN_PARKINGCARD: parkingcard ? 1 : 0
    };

    if (id != null) {
      map[DatabaseProvider.COLUMN_ID] = id;
    }

    return map;
  }

  static Vehicle fromMap(Map<String, dynamic> map) {
    return Vehicle(
        map[DatabaseProvider.COLUMN_ID],
        map[DatabaseProvider.COLUMN_KEY],
        map[DatabaseProvider.COLUMN_NAME],
        map[DatabaseProvider.COLUMN_LICENSEPLATE],
        double.parse(map[DatabaseProvider.COLUMN_WIDTH]),
        double.parse(map[DatabaseProvider.COLUMN_HEIGHT]),
        double.parse(map[DatabaseProvider.COLUMN_DEPTH]),
        double.parse(map[DatabaseProvider.COLUMN_TURNANGLE]),
        map[DatabaseProvider.COLUMN_NEAREXIT] == 1,
        map[DatabaseProvider.COLUMN_PARKINGCARD] == 1);
  }

  //Todo
  String toString() {}

  //Todo
  String setPreferences(bool nearexit, bool parkingcard) {}
}

class ElectricalVehicle extends Vehicle {
  bool docharge;
  String chargingprovider;
  DateTime chargebegin, chargeend;
  String charge;

  ElectricalVehicle(
      id,
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
      this.chargeend,
      this.charge)
      : super(id, key, name, licenseplate, width, height, depth, turnangle,
            nearexit, parkingcard);

  Map<String, dynamic> toMap() {
    var map = <String, dynamic>{
      DatabaseProvider.COLUMN_ID: id,
      DatabaseProvider.COLUMN_KEY: key,
      DatabaseProvider.COLUMN_NAME: name,
      DatabaseProvider.COLUMN_LICENSEPLATE: licenseplate,
      DatabaseProvider.COLUMN_WIDTH: width.toDouble(),
      DatabaseProvider.COLUMN_HEIGHT: height.toDouble(),
      DatabaseProvider.COLUMN_DEPTH: depth.toDouble(),
      DatabaseProvider.COLUMN_TURNANGLE: turnangle.toDouble(),
      DatabaseProvider.COLUMN_NEAREXIT: nearexit ? 1 : 0,
      DatabaseProvider.COLUMN_PARKINGCARD: parkingcard ? 1 : 0,
      DatabaseProvider.COLUMN_DOCHARGE: docharge ? 1 : 0,
      DatabaseProvider.COLUMN_CHARGINGPROVIDER: chargingprovider,
      DatabaseProvider.COLUMN_CHARGEBEGIN: chargebegin.toString(),
      DatabaseProvider.COLUMN_CHARGEEND: chargeend.toString(),
      DatabaseProvider.COLUMN_CHARGE: charge
    };

    if (id != null) {
      map[DatabaseProvider.COLUMN_ID] = id;
    }

    return map;
  }

  static ElectricalVehicle fromMap(Map<String, dynamic> map) {
    return ElectricalVehicle(
        map[DatabaseProvider.COLUMN_ID],
        map[DatabaseProvider.COLUMN_KEY],
        map[DatabaseProvider.COLUMN_NAME],
        map[DatabaseProvider.COLUMN_LICENSEPLATE],
        double.parse(map[DatabaseProvider.COLUMN_WIDTH]),
        double.parse(map[DatabaseProvider.COLUMN_HEIGHT]),
        double.parse(map[DatabaseProvider.COLUMN_DEPTH]),
        double.parse(map[DatabaseProvider.COLUMN_TURNANGLE]),
        map[DatabaseProvider.COLUMN_NEAREXIT] == 1,
        map[DatabaseProvider.COLUMN_PARKINGCARD] == 1,
        map[DatabaseProvider.COLUMN_DOCHARGE] == 1,
        map[DatabaseProvider.COLUMN_CHARGINGPROVIDER],
        DateTime.parse(map[DatabaseProvider.COLUMN_CHARGEBEGIN]),
        DateTime.parse(map[DatabaseProvider.COLUMN_CHARGEEND]),
        map[DatabaseProvider.COLUMN_CHARGE]);
  }

  //Todo
  String getBatteryCharge() {}
}
