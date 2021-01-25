class ExampleVehicle {
  String name;
  double width, height, depth, turnangle;

  ExampleVehicle(
      this.name, this.width, this.height, this.depth, this.turnangle);

  // static Vehicle fromJson(Map<String, dynamic> parsedJson) {
  //   List<int> dimensions = [
  //     int.parse(parsedJson['dimensionH']),
  //     int.parse(parsedJson['dimensionW']),
  //     int.parse(parsedJson['dimensionD'])
  //   ];
  //   List<bool> preferences = [
  //     parsedJson['nearExit'].parseBool(),
  //     parsedJson['parkingcard'].parseBool()
  //   ];
  //   return new Vehicle(parsedJson['key'], parsedJson['name'],
  //       parsedJson['licenseplate'], dimensions, preferences);
  // }

  //Todo
  String toString() {}
}
