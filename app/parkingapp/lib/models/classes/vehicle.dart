//serves as Interface for Vehicles
class Vehicle {
  int databaseId;

  String inAppKey, name, licensePlate;

  //Dimensions
  double height, width, length, turningCycle;

  //Preferences
  bool nearExitPreference, parkingCard;

  String setDimensions() {}

  String setPreferences() {}

  String toString() {}

  Map<String, dynamic> toMap() {}

  static Vehicle fromMap(Map<String, dynamic> map) {}
}
