import 'package:flutter/material.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkingGarage {
  String name;
  ParkingGarageType type;
  int freeParkingSpots;
  String image;
  String map;
  Coordinate bottomLeft, topRight;

  ParkingGarage(
      {this.name,
      this.type,
      this.freeParkingSpots,
      this.image,
      this.map,
      this.bottomLeft,
      this.topRight});

  int getFreeParkingSpots() {
    _updateFreeParkingSpots();
    return freeParkingSpots;
  }

  //sends the getFreeParkingSpots inquiry to the parking garage management system
  void _updateFreeParkingSpots() {
    //TODO add inquiry for parking spots
  }
}

class Coordinate {
  final double lattitude, longitude;
  Coordinate({@required this.lattitude, @required this.longitude});

  toString() {
    return lattitude.toString() + ', ' + longitude.toString();
  }
}
