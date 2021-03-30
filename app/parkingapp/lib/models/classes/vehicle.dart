import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/util/parkmanager.dart';

/// A Class that can't be instantiated.
abstract class Vehicle {
  int databaseId;

  String inAppKey, name, licensePlate;

  /// The Dimensions.
  double width, height, length, turningCycle, distRearAxleLicensePlate;

  /// The Preferences.
  bool nearExitPreference, parkingCard;

  bool parkedIn, parkingIn, parkingOut;

  /// Notifier for [parkedIn].
  ValueNotifier<bool> parkedInObserver;

  Map<String, dynamic> toMap() {
    var map = <String, dynamic>{
      DatabaseProvider.COLUMN_IN_APP_KEY: inAppKey,
      DatabaseProvider.COLUMN_NAME: name,
      DatabaseProvider.COLUMN_LICENSE_PLATE: licensePlate,
      DatabaseProvider.COLUMN_WIDTH: width.toDouble(),
      DatabaseProvider.COLUMN_HEIGHT: height.toDouble(),
      DatabaseProvider.COLUMN_LENGTH: length.toDouble(),
      DatabaseProvider.COLUMN_TURNING_CYCLE: turningCycle.toDouble(),
      DatabaseProvider.COLUMN_DIST_REAR_AXLE_LICENSE_PLATE:
          distRearAxleLicensePlate.toDouble(),
      DatabaseProvider.COLUMN_NEAR_EXIT_PREFERENCE: nearExitPreference ? 1 : 0,
      DatabaseProvider.COLUMN_PARKING_CARD: parkingCard ? 1 : 0,
      DatabaseProvider.COLUMN_PARKED_IN: parkedIn ? 1 : 0
    };

    // When [databaseId] exists, use the value
    if (databaseId != null) {
      map[DatabaseProvider.COLUMN_DATABASE_ID] = databaseId;
    }
    return map;
  }

  /// Sends a park in request to the [ParkManager].
  void parkIn(BuildContext context) {
    ParkManager.parkInRequest(context, this);
  }

  /// Sends a park out request to the [ParkManager].
  void parkOut(BuildContext context) {
    ParkManager.parkOutRequest(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateDatabaseID(BuildContext context, int id) {
    this.databaseId = id;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateInAppKey(BuildContext context, String key) {
    this.inAppKey = key;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateName(BuildContext context, String name) {
    this.name = name;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateLicensePlate(BuildContext context, String licensePlate) {
    this.licensePlate = licensePlate;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateWidth(BuildContext context, double width) {
    this.width = width;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateHeight(BuildContext context, double height) {
    this.height = height;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateLength(BuildContext context, double length) {
    this.length = length;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateTurningCycle(BuildContext context, double turningCycle) {
    this.turningCycle = turningCycle;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateDistRearAxleLicensePlate(BuildContext context, double distance) {
    this.distRearAxleLicensePlate = distance;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateNearExitPreference(BuildContext context, bool nearExitPreference) {
    this.nearExitPreference = nearExitPreference;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateParkingCard(BuildContext context, bool parkingCard) {
    this.parkingCard = parkingCard;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating and observer updating.
  void setAndUpdateParkedIn(BuildContext context, bool parkedIn) {
    this.parkedIn = parkedIn;
    this.parkedInObserver.value = this.parkedIn;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateParkIngIn(BuildContext context, bool parkingIn) {
    this.parkingIn = parkingIn;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter which includes database updating.
  void setAndUpdateParkIngOut(BuildContext context, bool parkingOut) {
    this.parkingOut = parkingOut;
    DataHelper.updateVehicle(context, this);
  }

  /// Setter for all dimensions which includes database updating.
  void setAndUpdateDimensions(BuildContext context, double width, double height,
      double length, double turningCycle, double distRearAxleLicensePlate) {
    this.width = width;
    this.height = height;
    this.length = length;
    this.turningCycle = turningCycle;
    this.distRearAxleLicensePlate = distRearAxleLicensePlate;
    DataHelper.updateVehicle(context, this);
  }

  /// Compares [vehicle] attribute values with own attribute values
  bool equals(Vehicle vehicle) {
    return this.inAppKey == vehicle.inAppKey &&
        this.databaseId == vehicle.databaseId &&
        this.name == vehicle.name &&
        this.licensePlate == vehicle.licensePlate &&
        this.width == vehicle.width &&
        this.height == vehicle.height &&
        this.length == vehicle.length &&
        this.turningCycle == vehicle.turningCycle &&
        this.distRearAxleLicensePlate == vehicle.distRearAxleLicensePlate &&
        this.nearExitPreference == vehicle.nearExitPreference &&
        this.parkingCard == vehicle.parkingCard &&
        this.parkedIn == vehicle.parkedIn &&
        this.parkingIn == vehicle.parkingIn &&
        this.parkingOut == vehicle.parkingOut;
  }
}
