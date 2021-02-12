import 'vehicleevent.dart';

class DeleteVehicle extends VehicleEvent {
  int vehicleIndex;

  DeleteVehicle(int index) {
    vehicleIndex = index;
  }
}
