import 'package:flutter/cupertino.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/bloc/events/updatevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';

import 'databaseprovider.dart';

/// helps accessing the bloc and database structure
/// to ensure consistency over bloc and database this first calls the database
/// and if this is successfull the bloc is updated aswell

class DataHelper {
  // adds a specific vehicle
  static addVehicle(BuildContext context, Vehicle vehicle) {
    DatabaseProvider.db.insert(vehicle).then((dbVehicle) {
      BlocProvider.of<VehicleBloc>(context).add(AddVehicle(dbVehicle));
    });
  }

  // deletes a specific vehicle
  static deleteVehicle(BuildContext context, Vehicle vehicle) {
    DatabaseProvider.db.delete(vehicle.inAppKey).then((_) {
      BlocProvider.of<VehicleBloc>(context).add(DeleteVehicle(vehicle));
    });
  }

  // updates a specific vehicle
  static updateVehicle(BuildContext context, Vehicle vehicle) {
    DatabaseProvider.db.update(vehicle).then((_) {
      BlocProvider.of<VehicleBloc>(context).add(UpdateVehicle(vehicle));
    });
  }

  // copies all vehicles from database to bloc (app start)
  static initVehicles(BuildContext context) {
    DatabaseProvider.db.getVehicles().then((dbVehicles) {
      BlocProvider.of<VehicleBloc>(context).add(SetVehicles(dbVehicles));
    });
  }
}
