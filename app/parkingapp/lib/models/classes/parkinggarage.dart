import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkingGarage {
  String name;
  ParkingGarageType type;
  int freeParkingSpots;
  String image;

  ParkingGarage(this.name, this.type, this.freeParkingSpots, this.image);
}
