import 'package:parkingapp/models/data/vehicles.dart';

class Vehicle {
  String key, name, licensePlate;

  //Dimensions
  int height, width, length, turningCycle;

  //Preferences
  bool nearExitPreference, parkingCard;

  String setDimensions() {}

  String setPreferences() {}

  String toString() {}
}
