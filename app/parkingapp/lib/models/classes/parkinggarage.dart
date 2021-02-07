import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkingGarage {
  String name;
  ParkingGarageType type;
  int freeParkingSpots;
  String image;
  List<String> chargingProviders;

  ParkingGarage(this.name, this.type, this.freeParkingSpots, this.image,
      this.chargingProviders);
}
