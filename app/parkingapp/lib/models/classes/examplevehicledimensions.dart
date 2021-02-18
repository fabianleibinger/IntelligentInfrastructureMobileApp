//singleton class for different vehicle dimensions
class ExampleVehicleDimensions {
  String name;

  double height, width, length, turningCycle;

  static List<ExampleVehicleDimensions> instance = [_audi, _bmw, _tesla];

  static ExampleVehicleDimensions _audi =
      ExampleVehicleDimensions._('Audi', 1233.93, 2934.23, 4529.30, 0);
  static ExampleVehicleDimensions _bmw =
      ExampleVehicleDimensions._('BMW', 1643.93, 2745.23, 4463.30, 0);
  static ExampleVehicleDimensions _tesla =
      ExampleVehicleDimensions._('Tesla', 1223.93, 2934.23, 4529.3, 0);

  //private constructor
  ExampleVehicleDimensions._(
      this.name, this.height, this.width, this.length, this.turningCycle);
}
