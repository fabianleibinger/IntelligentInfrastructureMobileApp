import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
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

  ParkingGarage({
    this.name,
    this.type,
    this.freeParkingSpots,
    this.freeChargeableParkingSpots,
    this.image,
    this.map,
    this.bottomLeft,
    this.topRight,
  }) {
    this.updateAllFreeParkingSpots();
  }

  //updates all parking spots
  void updateAllFreeParkingSpots() {
    this.updateFreeParkingSpots();
    this.updateFreeChargeableParkingSpots();
  }

  //sends the getFreeParkingSpots inquiry to the parking garage management system
  //updates freeParkingSpots
  void updateFreeParkingSpots() {
    ApiProvider.getFreeParkingSpots().then((value) =>
    this.freeParkingSpots =
        int.parse(value.values.single
            .toString()
            .split(' ')
            .last));
  }

  //sends the getFreeChargeableParkingSpots inquiry to the parking garage management system
  //updates freeParkingSpots
  void updateFreeChargeableParkingSpots() {
    ApiProvider.getFreeChargeableParkingSpots().then((value) =>
    this.freeChargeableParkingSpots =
        int.parse(value.values.single
            .toString()
            .split(' ')
            .last));
  }

  //returns true when spots for specific preferences are available
  bool vehicleSpecificSpotsAvailable(Vehicle vehicle) {
    return this.getFreeSpotsForVehicle(vehicle) > 0;
  }

  //vehicles preferences decide if chargeable or normal parking spots are returned
  int getFreeSpotsForVehicle(Vehicle vehicle) {
    if (vehicle.runtimeType == ChargeableVehicle) {
      return this._chooseParkingSpotsForChargeableVehicle(vehicle);
    } else {
      return this.freeParkingSpots;
    }
  }

  //if vehicle is supposed to load return chargeable parking spots, otherwise don't
  _chooseParkingSpotsForChargeableVehicle(ChargeableVehicle vehicle) {
    if (vehicle.doCharge) {
      return this.freeChargeableParkingSpots;
    } else {
      return this.freeParkingSpots;
    }
  }
}

class Coordinate {
  final double lattitude, longitude;
  Coordinate({@required this.lattitude, @required this.longitude});
  toString() {
    return lattitude.toString() + ', ' + longitude.toString();
  }
}
