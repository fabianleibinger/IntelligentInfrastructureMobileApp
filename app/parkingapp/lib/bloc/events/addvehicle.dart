import 'package:parkingapp/models/classes/vehicle.dart';

import 'vehicleevent.dart';

class AddVehicle extends VehicleEvent {
  Vehicle newVehicle;

  AddVehicle(Vehicle vehicle) {
    newVehicle = vehicle;
  }
}
