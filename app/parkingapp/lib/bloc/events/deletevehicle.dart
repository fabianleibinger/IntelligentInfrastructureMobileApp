import 'package:parkingapp/models/classes/vehicle.dart';

import 'vehicleevent.dart';

class DeleteVehicle extends VehicleEvent {
  Vehicle eventVehicle;

  DeleteVehicle(Vehicle vehicle) {
    eventVehicle = vehicle;
  }
}
