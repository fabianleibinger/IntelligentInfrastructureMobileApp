import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkingGarage {
  String name;
  ParkingGarageType type;
  int freeParkingSpots;
  String image;

  ParkingGarage(this.name, this.type, this.freeParkingSpots, this.image);

  int getFreeParkingSpots() {
    _updateFreeParkingSpots();
    return freeParkingSpots;
  }

  void _updateFreeParkingSpots() {
    //TODO add inquiry for parking spots
  }
}
