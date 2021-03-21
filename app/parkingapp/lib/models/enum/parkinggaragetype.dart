enum ParkingGarageType {
  Parkgarage,
  Parkplatz,
  Tiefgarage
}

/// Returns the enum value as a string.
extension ParseToString on ParkingGarageType {
  String toShortString() {
    return this.toString().split('.').last;
  }
}