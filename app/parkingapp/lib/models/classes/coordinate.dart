import 'package:flutter/material.dart';

class Coordinate {
  final double lattitude, longitude;

  Coordinate({@required this.lattitude, @required this.longitude});

  toString() {
    return lattitude.toString() + ', ' + longitude.toString();
  }
}
