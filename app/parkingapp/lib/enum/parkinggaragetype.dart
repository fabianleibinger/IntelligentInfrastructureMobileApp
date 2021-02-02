enum ParkingGarageType {
  Parkgarage,
  Parkplatz,
  Tiefgarage
}

extension ParseToString on ParkingGarageType {
  String toShortString() {
    return this.toString().split('.').last;
  }
}