import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';

class StandardVehicle extends Vehicle {
  StandardVehicle(
      this.inAppKey,
      this.name,
      this.licensePlate,
      this.width,
      this.height,
      this.length,
      this.turningCycle,
      this.distRearAxleLicensePlate,
      this.nearExitPreference,
      this.parkingCard,
      this.parkedIn) {
    this.parkedInObserver = ValueNotifier(this.parkedIn);
    this.parkingIn = false;
    this.parkingOut = false;
  }

  //private constructor: only called by fromMap() method, database defines databaseId
  StandardVehicle._(
      this.databaseId,
      this.inAppKey,
      this.name,
      this.licensePlate,
      this.width,
      this.height,
      this.length,
      this.turningCycle,
      this.distRearAxleLicensePlate,
      this.nearExitPreference,
      this.parkingCard,
      this.parkedIn) {
    this.parkedInObserver = ValueNotifier(this.parkedIn);
    this.parkingIn = false;
    this.parkingOut = false;
  }

  static Vehicle fromMap(Map<String, dynamic> map) {
    return StandardVehicle._(
        map[DatabaseProvider.COLUMN_DATABASE_ID],
        map[DatabaseProvider.COLUMN_IN_APP_KEY],
        map[DatabaseProvider.COLUMN_NAME],
        map[DatabaseProvider.COLUMN_LICENSE_PLATE],
        map[DatabaseProvider.COLUMN_WIDTH],
        map[DatabaseProvider.COLUMN_HEIGHT],
        map[DatabaseProvider.COLUMN_LENGTH],
        map[DatabaseProvider.COLUMN_TURNING_CYCLE],
        map[DatabaseProvider.COLUMN_DIST_REAR_AXLE_LICENSE_PLATE],
        map[DatabaseProvider.COLUMN_NEAR_EXIT_PREFERENCE] == 1,
        map[DatabaseProvider.COLUMN_PARKING_CARD] == 1,
        map[DatabaseProvider.COLUMN_PARKED_IN] == 1);
  }

  @override
  int databaseId;

  @override
  String inAppKey, name, licensePlate;

  @override
  double width, height, length, turningCycle, distRearAxleLicensePlate;

  @override
  bool nearExitPreference, parkingCard;

  @override
  bool parkedIn;

  Vehicle toElectricVehicle() {
    return ChargeableVehicle(
        inAppKey,
        name,
        licensePlate,
        height,
        width,
        length,
        turningCycle,
        distRearAxleLicensePlate,
        nearExitPreference,
        parkingCard,
        false,
        false,
        '',
        TimeOfDay(hour: 0, minute: 0),
        TimeOfDay(hour: 23, minute: 59));
  }
}
