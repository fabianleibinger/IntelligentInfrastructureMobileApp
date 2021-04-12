import 'package:flutter/material.dart';

/// Stores a longitude and latitude from a geolocation
class Coordinate {
  final double latitude, longitude;

  Coordinate({@required this.latitude, @required this.longitude});

  toString() {
    return latitude.toString() + ', ' + longitude.toString();
  }
}
