import 'package:parkingapp/models/classes/vehicle.dart';

import 'vehicleevent.dart';

class AddVehicle extends VehicleEvent {
  Vehicle eventVehicle;

  AddVehicle(Vehicle vehicle) {
    eventVehicle = vehicle;
  }
}
