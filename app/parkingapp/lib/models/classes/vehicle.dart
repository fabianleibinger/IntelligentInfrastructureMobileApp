abstract class Vehicle {
  int databaseId;

  String inAppKey, name, licensePlate;

  //Dimensions
  double height, width, length, turningCycle;

  //Preferences
  bool nearExitPreference, parkingCard;

  String setDimensions() {}

  String setPreferences() {}

  String toString() {}
}
