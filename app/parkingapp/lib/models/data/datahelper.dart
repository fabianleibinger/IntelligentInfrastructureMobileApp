import 'package:flutter/cupertino.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/bloc/events/updatevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';

import 'databaseprovider.dart';

class DataHelper {
  static addVehicle(BuildContext context, Vehicle vehicle) {
    DatabaseProvider.db.insert(vehicle).then((dbvehicle) {
      BlocProvider.of<VehicleBloc>(context).add(AddVehicle(dbvehicle));
    });
  }

  static deleteVehicle(BuildContext context, Vehicle vehicle) {
    DatabaseProvider.db.delete(vehicle.databaseId).then((_) {
      BlocProvider.of<VehicleBloc>(context).add(DeleteVehicle(vehicle));
    });
  }

  static updateVehicle(BuildContext context, Vehicle vehicle) {
    DatabaseProvider.db.update(vehicle).then((_) {
      BlocProvider.of<VehicleBloc>(context).add(UpdateVehicle(vehicle));
    });
  }

  static initVehicles(BuildContext context) {
    DatabaseProvider.db.getVehicles().then((dbVehicles) {
      BlocProvider.of<VehicleBloc>(context).add(SetVehicles(dbVehicles));
    });
  }
}
