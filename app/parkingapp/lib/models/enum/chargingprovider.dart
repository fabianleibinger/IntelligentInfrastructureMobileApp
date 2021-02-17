enum ChargingProvider {
  Automatisch,
  eon,
  EnBW,
}

extension ParseToString on ChargingProvider {
  String toShortString() {
    return this.toString().split('.').last;
  }
}