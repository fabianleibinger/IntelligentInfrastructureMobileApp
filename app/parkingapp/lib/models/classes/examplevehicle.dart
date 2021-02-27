import 'package:flutter/material.dart';

class ExampleVehicle {
  String name;
  double width, height, length, turningCycle;

  ExampleVehicle(this.name, this.width, this.height, this.length,
      this.turningCycle);

  static ExampleVehicle fromJson(Map<String, dynamic> parsedJson) {
    return new ExampleVehicle(
        parsedJson['name'], parsedJson['width'], parsedJson['height'],
        parsedJson['length'], parsedJson['turningCycle']);
  }
}
