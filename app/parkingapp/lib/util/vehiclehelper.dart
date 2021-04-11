import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/util/utility.dart';

final double notSpecifiedDouble = 0;
final String notSpecifiedString = '';
final bool notSpecifiedBool = false;
final TimeOfDay notSpecifiedTimeOfDay = TimeOfDay(hour: 0, minute: 0);
final String defaultChargingProvider =
    ChargingProvider.Automatisch.toShortString();

class VehicleHelper {
  static void updateMainPageVehicle(
      {@required BuildContext context, Vehicle parseVehicle}) {
    if (parseVehicle == null) {
      print('set new electric vehicle on main page');
      vehicle = ChargeableVehicle(
        Utility.generateKey(),
        notSpecifiedString,
        notSpecifiedString,
        notSpecifiedDouble,
        notSpecifiedDouble,
        notSpecifiedDouble,
        notSpecifiedDouble,
        notSpecifiedDouble,
        notSpecifiedBool,
        notSpecifiedBool,
        notSpecifiedBool,
        notSpecifiedBool,
        defaultChargingProvider,
        notSpecifiedTimeOfDay,
        notSpecifiedTimeOfDay,
      );
      print('adding dummy vehicle to database');
      DataHelper.addVehicle(context, vehicle);
    } else if (parseVehicle.runtimeType == ChargeableVehicle) {
      print('parsedVehicle is electric; using parsed vehicle as is');
      vehicle = parseVehicle;
    } else if (parseVehicle.runtimeType == StandardVehicle) {
      vehicle = parseVehicle;
      DataHelper.deleteVehicle(context, vehicle);
      print('parsedVehicle is standard; converting into chargeable');
      StandardVehicle convertVehicle = parseVehicle;
      vehicle = convertVehicle.toChargeableVehicle();
      print('converting standard vehicle to electric vehicle in database');
      DataHelper.addVehicle(context, vehicle);
    }
  }

  static Future<bool> cleanUpDummy(
      {@required BuildContext context, Vehicle parseVehicle}) async {
    print('starting clean up');
    if (parseVehicle == null) {
      print('removing dummy vehicle');
      //the behicle did not exist before opening the form and needs to be removed
      DataHelper.deleteVehicle(context, vehicle);
    } else {
      print('restoring previous state');
      print('previous vehicle: inAppID: ' +
          parseVehicle.inAppKey +
          ' name: ' +
          parseVehicle.name +
          ' licensePlate: ' +
          parseVehicle.licensePlate +
          ' parkpreferences: ' +
          parseVehicle.nearExitPreference.toString() +
          parseVehicle.parkingCard.toString());
      print('temporary vehicle: inAppID: ' +
          vehicle.inAppKey +
          ' name: ' +
          vehicle.name +
          ' licensePlate: ' +
          vehicle.licensePlate +
          ' parkpreferences: ' +
          vehicle.nearExitPreference.toString() +
          vehicle.parkingCard.toString());
      //restore the previous state
      DataHelper.updateVehicle(context, parseVehicle);
    }
    return true;
  }

  ///deletes all [Vehicle] from the Database that are dummy vehicles
  static void cleanUpVehicles(BuildContext context) {
    for (Vehicle vehicle in BlocProvider.of<VehicleBloc>(context).state) {
      if (vehicle.name == notSpecifiedString &&
          vehicle.licensePlate == notSpecifiedString) {
        DataHelper.deleteVehicle(context, vehicle);
      }
    }
  }
}
