import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/vehicles.dart';

class LoadableVehicle implements Vehicle {
  bool doCharge;
  String chargingProvider;
  DateTime chargeTimeBegin, chargeTimeEnd;
  String charge;

  LoadableVehicle(
      this.key,
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

  LoadableVehicle._(
      this.id,
      this.key,
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

  Map<String, dynamic> toMap() {
    var map = <String, dynamic>{
      DatabaseProvider.COLUMN_ID: id,
      DatabaseProvider.COLUMN_KEY: key,
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
      DatabaseProvider.COLUMN_CHARGE_TIME_BEGIN: chargeTimeBegin.toString(),
      DatabaseProvider.COLUMN_CHARGE_TIME_END: chargeTimeEnd.toString(),
      DatabaseProvider.COLUMN_CHARGE: charge
    };

    if (id != null) {
      map[DatabaseProvider.COLUMN_ID] = id;
    }

    return map;
  }

  static LoadableVehicle fromMap(Map<String, dynamic> map) {
    return LoadableVehicle._(
        map[DatabaseProvider.COLUMN_ID],
        map[DatabaseProvider.COLUMN_KEY],
        map[DatabaseProvider.COLUMN_NAME],
        map[DatabaseProvider.COLUMN_LICENSE_PLATE],
        double.parse(map[DatabaseProvider.COLUMN_WIDTH]),
        double.parse(map[DatabaseProvider.COLUMN_HEIGHT]),
        double.parse(map[DatabaseProvider.COLUMN_LENGTH]),
        double.parse(map[DatabaseProvider.COLUMN_TURNING_CYCLE]),
        map[DatabaseProvider.COLUMN_NEAR_EXIT_PREFERENCE] == 1,
        map[DatabaseProvider.COLUMN_PARKING_CARD] == 1,
        map[DatabaseProvider.COLUMN_DO_CHARGE] == 1,
        map[DatabaseProvider.COLUMN_CHARGING_PROVIDER],
        DateTime.parse(map[DatabaseProvider.COLUMN_CHARGE_TIME_BEGIN]),
        DateTime.parse(map[DatabaseProvider.COLUMN_CHARGE_TIME_END]),
        map[DatabaseProvider.COLUMN_CHARGE]);
  }


  String getBatteryCharge() {
    // TODO: implement getBatteryCharge
    throw UnimplementedError();
  }

  @override
  String setDimensions() {
    // TODO: implement setDimensions
    throw UnimplementedError();
  }

  @override
  String setPreferences() {
    // TODO: implement setPreferences
    throw UnimplementedError();
  }

  @override
  int id;

  @override
  String key, name, licensePlate;

  @override
  double height, width, length, turningCycle;

  @override
  bool nearExitPreference, parkingCard;
}
