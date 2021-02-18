import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/data/datahelper.dart';

class LoadableVehicle extends Vehicle {
  bool doCharge;
  String chargingProvider;
  TimeOfDay chargeTimeBegin, chargeTimeEnd;
  String charge;

  LoadableVehicle(
      this.inAppKey,
      this.name,
      this.licensePlate,
      this.height,
      this.width,
      this.length,
      this.turningCycle,
      this.nearExitPreference,
      this.parkingCard,
      this.doCharge,
      this.chargingProvider,
      this.chargeTimeBegin,
      this.chargeTimeEnd,
      this.charge);

  //private constructor: only called by fromMap() method, database defines databaseId
  LoadableVehicle._(
      this.databaseId,
      this.inAppKey,
      this.name,
      this.licensePlate,
      this.height,
      this.width,
      this.length,
      this.turningCycle,
      this.nearExitPreference,
      this.parkingCard,
      this.doCharge,
      this.chargingProvider,
      this.chargeTimeBegin,
      this.chargeTimeEnd,
      this.charge);

  @override
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
      DatabaseProvider.COLUMN_PARKING_CARD: parkingCard ? 1 : 0,
      DatabaseProvider.COLUMN_DO_CHARGE: doCharge ? 1 : 0,
      DatabaseProvider.COLUMN_CHARGING_PROVIDER: chargingProvider,
      DatabaseProvider.COLUMN_CHARGE_TIME_BEGIN:
          chargeTimeBegin.toString().split('(').last.split(')').first,
      DatabaseProvider.COLUMN_CHARGE_TIME_END:
          chargeTimeEnd.toString().split('(').last.split(')').first,
      DatabaseProvider.COLUMN_CHARGE: charge
    };

    if (databaseId != null) {
      map[DatabaseProvider.COLUMN_DATABASE_ID] = databaseId;
    }

    return map;
  }

  static LoadableVehicle fromMap(Map<String, dynamic> map) {
    String timeBegin = map[DatabaseProvider.COLUMN_CHARGE_TIME_BEGIN];
    String timeEnd = map[DatabaseProvider.COLUMN_CHARGE_TIME_END];
    return LoadableVehicle._(
        map[DatabaseProvider.COLUMN_DATABASE_ID],
        map[DatabaseProvider.COLUMN_IN_APP_KEY],
        map[DatabaseProvider.COLUMN_NAME],
        map[DatabaseProvider.COLUMN_LICENSE_PLATE],
        map[DatabaseProvider.COLUMN_WIDTH],
        map[DatabaseProvider.COLUMN_HEIGHT],
        map[DatabaseProvider.COLUMN_LENGTH],
        map[DatabaseProvider.COLUMN_TURNING_CYCLE],
        map[DatabaseProvider.COLUMN_NEAR_EXIT_PREFERENCE] == 1,
        map[DatabaseProvider.COLUMN_PARKING_CARD] == 1,
        map[DatabaseProvider.COLUMN_DO_CHARGE] == 1,
        map[DatabaseProvider.COLUMN_CHARGING_PROVIDER],
        TimeOfDay(
            hour: int.parse(timeBegin.split(':').first),
            minute: int.parse(timeBegin.split(':').last)),
        TimeOfDay(
            hour: int.parse(timeEnd.split(':').first),
            minute: int.parse(timeEnd.split(':').last)),
        map[DatabaseProvider.COLUMN_CHARGE]);
  }

  //setter which includes database updating
  void setDoCharge(BuildContext context, bool doCharge) {
    this.doCharge = doCharge;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setChargingProvider(BuildContext context, String chargingProvider) {
    this.chargingProvider = chargingProvider;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setChargeTimeBegin(BuildContext context, TimeOfDay begin) {
    this.chargeTimeBegin = begin;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setChargeTimeEnd(BuildContext context, TimeOfDay end) {
    this.chargeTimeEnd = end;
    DataHelper.updateVehicle(context, this);
  }

  //setter which includes database updating
  void setCharge(BuildContext context, String charge) {
    this.charge = charge;
    DataHelper.updateVehicle(context, this);
  }

  @override
  int databaseId;

  @override
  String inAppKey, name, licensePlate;

  @override
  double height, width, length, turningCycle;

  @override
  bool nearExitPreference, parkingCard;

  StandardVehicle toStandardVehicle() {
    return StandardVehicle(inAppKey, name, licensePlate, height, width, length,
        turningCycle, nearExitPreference, parkingCard);
  }
}
