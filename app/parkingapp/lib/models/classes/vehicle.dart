import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/coordinate.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/util/parkmanager.dart';

//cannot be instantiated
abstract class Vehicle {
  int databaseId;

  String inAppKey, name, licensePlate;

  //Dimensions
  double width, height, length, turningCycle, distRearAxleLicensePlate;

  //Preferences
  bool nearExitPreference, parkingCard;

  bool parkedIn, parkingIn, parkingOut;

  //observer for parkedIn
  ValueNotifier<bool> parkedInObserver;

  Coordinate parkingSpot, location;

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

    if (databaseId != null) {
      map[DatabaseProvider.COLUMN_DATABASE_ID] = databaseId;
    }
    return map;
  }

  //sends a parkIn request to the park manager
  void parkIn(BuildContext context) {
    ParkManager.parkInRequest(context, this);
  }

  //sends a parkIn request to the park manager
  void parkOut(BuildContext context) {
    ParkManager.parkOutRequest(context, this);
  }

  //setter which includes database updating
  void setDatabaseID(BuildContext context, int id) {
    this.databaseId = id;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setInAppKey(BuildContext context, String key) {
    this.inAppKey = key;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setName(BuildContext context, String name) {
    this.name = name;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setLicensePlate(BuildContext context, String licensePlate) {
    this.licensePlate = licensePlate;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setWidth(BuildContext context, double width) {
    this.width = width;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setHeight(BuildContext context, double height) {
    this.height = height;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setLength(BuildContext context, double length) {
    this.length = length;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setTurningCycle(BuildContext context, double turningCycle) {
    this.turningCycle = turningCycle;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setDistRearAxleLicensePlate(BuildContext context, double distance) {
    this.distRearAxleLicensePlate = distance;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setNearExitPreference(BuildContext context, bool nearExitPreference) {
    this.nearExitPreference = nearExitPreference;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setParkingCard(BuildContext context, bool parkingCard) {
    this.parkingCard = parkingCard;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating and observer updating
  void setParkedIn(BuildContext context, bool parkedIn) {
    this.parkedIn = parkedIn;
    this.parkedInObserver.value = this.parkedIn;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating and observer updating
  void setParkIngIn(BuildContext context, bool parkingIn) {
    this.parkingIn = parkingIn;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating and observer updating
  void setParkIngOut(BuildContext context, bool parkingOut) {
    this.parkingOut = parkingOut;
    this.parkingSpot = null;
    DataHelper.updateVehicle(context, this);
  }

  //setter for all dimensions which includes database updating
  void setDimensions(BuildContext context, double width, double height,
      double length, double turningCycle, double distRearAxleLicensePlate) {
    this.width = width;
    this.height = height;
    this.length = length;
    this.turningCycle = turningCycle;
    this.distRearAxleLicensePlate = distRearAxleLicensePlate;
    DataHelper.updateVehicle(context, this);
  }

  //set the parkingSpot for the vehicle
  void setParkingSpot(Coordinate parkingSpot) {
    this.parkingSpot = parkingSpot;
  }
}
