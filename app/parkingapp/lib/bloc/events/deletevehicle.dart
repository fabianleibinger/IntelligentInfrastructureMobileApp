import 'package:parkingapp/models/classes/vehicle.dart';

import 'vehicleevent.dart';

class DeleteVehicle extends VehicleEvent {
  Vehicle vehicleIndex;

  DeleteVehicle(Vehicle index) {
    vehicleIndex = index;
  }
}
