import 'package:flutter/material.dart';

class ExampleVehicle {
  String name;
  double height, width, length;

  ExampleVehicle(this.name, this.height, this.width, this.length);

  static ExampleVehicle fromJson(Map<String, dynamic> parsedJson) {
    return new ExampleVehicle(parsedJson['name'], parsedJson['height'],
        parsedJson['width'], parsedJson['length']);
  }
}
