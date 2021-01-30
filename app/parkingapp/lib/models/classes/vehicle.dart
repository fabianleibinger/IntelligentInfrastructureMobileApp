//Interface for vehicles
class Vehicle {
  int id;

  String key, name, licensePlate;

  //Dimensions
  double height, width, length, turningCycle;

  //Preferences
  bool nearExitPreference, parkingCard;

  String setDimensions() {}

  String setPreferences() {}

  String toString() {}
}
