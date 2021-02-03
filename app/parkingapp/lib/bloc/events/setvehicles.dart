import 'package:parkingapp/models/classes/vehicle.dart';

import 'vehicleevent.dart';

class SetVehicles extends VehicleEvent {
  List<Vehicle> vehicleList;

  SetVehicles(List<Vehicle> vehicles) {
    vehicleList = vehicles;
  }
}
