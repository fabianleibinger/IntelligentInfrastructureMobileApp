import 'package:parkingapp/models/classes/vehicle.dart';

import 'vehicleevent.dart';

class UpdateVehicle extends VehicleEvent {
  Vehicle eventVehicle;

  UpdateVehicle(Vehicle vehicle) {
    eventVehicle = vehicle;
  }
}
