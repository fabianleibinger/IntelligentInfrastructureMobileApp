import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/coordinate.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkingGarage {
  String name;
  ParkingGarageType type;
  int freeParkingSpots;
  int freeChargeableParkingSpots;
  String image;
  String map;
  Coordinate bottomLeft, topRight;
  int mapHeight, mapWidth;

  /// The Constructor that updates parking spots.
  ParkingGarage(
      {this.name,
      this.type,
      this.freeParkingSpots,
      this.freeChargeableParkingSpots,
      this.image,
      this.map,
      this.bottomLeft,
      this.topRight,
      this.mapHeight,
      this.mapWidth}) {
    this.updateAllFreeParkingSpots();
  }

  /// Updates all parking spots.
  void updateAllFreeParkingSpots() {
    this.updateFreeParkingSpots();
    this.updateFreeChargeableParkingSpots();
  }

  /// Updates [freeParkingSpots]
  /// by [ApiProvider.getFreeParkingSpots()].
  void updateFreeParkingSpots() {
    ApiProvider.getFreeParkingSpots()
        .then((value) => this.freeParkingSpots =
            int.parse(value.values.single.toString().split(' ').last))
        .catchError((e) => this.freeParkingSpots = 0);
  }

  /// Updates [freeChargeableParkingSpots]
  /// by [ApiProvider.getFreeChargeableParkingSpots()].
  void updateFreeChargeableParkingSpots() {
    ApiProvider.getFreeChargeableParkingSpots()
        .then((value) => this.freeChargeableParkingSpots =
            int.parse(value.values.single.toString().split(' ').last))
        .catchError((e) => this.freeChargeableParkingSpots = 0);
  }

  /// Returns true when parking spots
  /// for specific [vehicle] preferences are available.
  bool vehicleSpecificSpotsAvailable(Vehicle vehicle) {
    return this.getFreeSpotsForVehicle(vehicle) > 0;
  }

  /// Returns [freeParkingSpots] or [freeChargeableParkingSpots]
  /// according to [vehicle].
  int getFreeSpotsForVehicle(Vehicle vehicle) {
    if (vehicle.runtimeType == ChargeableVehicle) {
      return this._chooseParkingSpotsForChargeableVehicle(vehicle);
    } else {
      return this.freeParkingSpots;
    }
  }

  /// Returns [freeParkingSpots] or [freeChargeableParkingSpots]
  /// according to [vehicle.doCharge].
  _chooseParkingSpotsForChargeableVehicle(ChargeableVehicle vehicle) {
    if (vehicle.doCharge) {
      return this.freeChargeableParkingSpots;
    } else {
      return this.freeParkingSpots;
    }
  }
}
