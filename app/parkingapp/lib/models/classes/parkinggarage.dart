import 'package:flutter/material.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkingGarage {
  String name;
  ParkingGarageType type;
  int freeParkingSpots;
  String image;

  ParkingGarage({this.name, this.type, this.freeParkingSpots, this.image});

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
