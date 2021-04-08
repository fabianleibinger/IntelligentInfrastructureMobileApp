import 'package:flutter/material.dart';

class Coordinate {
  final double latitude, longitude;

  Coordinate({@required this.latitude, @required this.longitude});

  toString() {
    return latitude.toString() + ', ' + longitude.toString();
  }
}
