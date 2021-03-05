import 'package:flutter/material.dart';

class ExampleVehicle {
  String name;
  double height, width, length, turningCycle;

  ExampleVehicle(this.name, this.height, this.width, this.length, turningCycle);

  static ExampleVehicle fromJson(Map<String, dynamic> parsedJson) {
    return new ExampleVehicle(parsedJson['name'], parsedJson['height'],
        parsedJson['width'], parsedJson['length'], parsedJson['turningCycle']);
  }
}
