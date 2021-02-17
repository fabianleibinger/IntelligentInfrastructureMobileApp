//serves as Interface for Vehicles
import 'package:flutter/material.dart';

class Vehicle {
  int databaseId;

  String inAppKey, name, licensePlate;

  //Dimensions
  double height, width, length, turningCycle;

  //Preferences
  bool nearExitPreference, parkingCard;

  void setDimensions(
      BuildContext context, double height, double width, double length) {}

  String setPreferences() {}

  String toString() {}

  Map<String, dynamic> toMap() {}

  static Vehicle fromMap(Map<String, dynamic> map) {}
}
