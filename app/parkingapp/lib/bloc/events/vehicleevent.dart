import 'dart:html';

import 'package:parkingapp/models/classes/vehicle.dart';

enum EventType { add, delete, modify }

class VehicleEvent {
  Vehicle vehicle;
  int vehicleIndex;
  EventType eventType;

  VehicleEvent.add(Vehicle vehicle) {
    this.eventType = EventType.add;
    this.vehicle = vehicle;
  }

  VehicleEvent.delete(int index) {
    this.eventType = EventType.delete;
    this.vehicleIndex = index;
  }

  VehicleEvent.modify(int index, Vehicle vehicle) {
    this.eventType = EventType.modify;
    this.vehicleIndex = index;
  }
}
