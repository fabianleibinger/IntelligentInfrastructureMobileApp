import 'package:parkingapp/models/data/databaseprovider.dart';

abstract class Vehicle {
  int databaseId;

  String inAppKey, name, licensePlate;

  //Dimensions
  double height, width, length, turningCycle;

  //Preferences
  bool nearExitPreference, parkingCard;

  Map<String, dynamic> toMap() {
    var map = <String, dynamic>{
      DatabaseProvider.COLUMN_IN_APP_KEY: inAppKey,
      DatabaseProvider.COLUMN_NAME: name,
      DatabaseProvider.COLUMN_LICENSE_PLATE: licensePlate,
      DatabaseProvider.COLUMN_WIDTH: width.toDouble(),
      DatabaseProvider.COLUMN_HEIGHT: height.toDouble(),
      DatabaseProvider.COLUMN_LENGTH: length.toDouble(),
      DatabaseProvider.COLUMN_TURNING_CYCLE: turningCycle.toDouble(),
      DatabaseProvider.COLUMN_NEAR_EXIT_PREFERENCE: nearExitPreference ? 1 : 0,
      DatabaseProvider.COLUMN_PARKING_CARD: parkingCard ? 1 : 0
    };

    if (databaseId != null) {
      map[DatabaseProvider.COLUMN_DATABASE_ID] = databaseId;
    }

    return map;
  }

  void setDatabaseID(int id) {
    this.databaseId = id;
  }

  void setInAppKey(String key) {
    this.inAppKey = key;
  }

  void setName(String name) {
    this.name = name;
  }

  void setLicensePlate(String licensePlate) {
    this.licensePlate = licensePlate;
  }

  void setHeight(double height) {
    this.height = height;
  }

  void setWidth(double width) {
    this.width = width;
  }

  void setLength(double length) {
    this.length = length;
  }

  void setTurningCycle(double turningCycle) {
    this.turningCycle = turningCycle;
  }

  void setNearExitPreference(bool nearExitPreference) {
    this.nearExitPreference = nearExitPreference;
  }

  void setParkingCard(bool parkingCard) {
    this.parkingCard = parkingCard;
  }
}
