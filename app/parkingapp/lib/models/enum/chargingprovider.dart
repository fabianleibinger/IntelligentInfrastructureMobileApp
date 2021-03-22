enum ChargingProvider {
  Automatisch,
  eon,
  EnBW,
}

/// Returns the enum value as a string.
extension ParseToString on ChargingProvider {
  String toShortString() {
    return this.toString().split('.').last;
  }
}