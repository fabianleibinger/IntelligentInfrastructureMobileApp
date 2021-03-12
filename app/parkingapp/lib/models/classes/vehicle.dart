import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/dialogs/noconnectiondialog.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkinggarageoccupieddialog.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';

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

  static Vehicle fromMap(Map<String, dynamic> map) {}

  //sends the park in inquiry to the parking garage management system
  void parkIn(BuildContext context) {
    //check if vehicle needs to be parked in
    if (!this.parkedIn && !this.parkingIn) {
      //try to park in
      if (currentParkingGarage.getFreeSpotsForVehicle(this) > 0) {
        //vehicle parking in
        this.setParkIngIn(context, true);
        print(this.name + ' wird eingeparkt');

        //try to contact server
        ApiProvider.parkIn(this).then((value) {
          this.setParkedIn(context, true);
          print('vehicle parked in: ' + this.parkedIn.toString());

          //vehicle not parking in anymore
        }).whenComplete(() {
          this.setParkIngIn(context, false);
          //if park in didn't work: connection to server failed
          if (!this.parkedIn) {
            NoConnectionDialog.createDialog(context);
          }
        });
      } else {
        //no parking spots available
        print('no parking spots available');
        ParkingGarageOccupiedDialog.createDialog(context);
      }
    } else {
      //vehicle is already parked in
      print('vehicle ' + this.name + ' is already parked in');
    }
  }

  //sends the park out inquiry to the parking garage management system
  void parkOut(BuildContext context) {
    //check if vehicle needs to be parked in
    if (!this.parkingOut) {
      //cancelling park in if needed
      this.setParkIngIn(context, false);
      //park out or cancel park in process
      this.setParkIngOut(context, true);
      print(this.name + ' wird ausgeparkt');

      //try to contact server
      ApiProvider.parkOut(this).then((value) {
        this.setParkedIn(context, false);
        print('vehicle parked in: ' + this.parkedIn.toString());
        //open main page
        Navigator.pushReplacementNamed(context, this.inAppKey);
        ParkDialog.createParkOutFinishedDialog(context);

        //vehicle not parking out anymore
      }).whenComplete(() {
        this.setParkIngOut(context, false);
        //if park out didn't work: connection to server failed
        if (this.parkedIn) {
          NoConnectionDialog.createDialog(context);
        }
      });
    }
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
}
