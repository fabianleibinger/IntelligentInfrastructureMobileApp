import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/ui/parkpage/parkpage.dart';
import 'package:parkingapp/ui/parkpages/parkinpage.dart';
import 'package:parkingapp/ui/parkpages/parkoutpage.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';

class Routes {
  static const String main = MainPage.routeName;
  static const String vehicle = VehiclePage.routeName;
  static const String settings = SettingsPage.routeName;
  static const String agb = AGB.routeName;
  static const String park = ParkPage.routeName;
  static const String parkIn = ParkInPage.routeName;
  static const String parkOut = ParkOutPage.routeName;
  static const String createVehicle = CreateVehicle.routeName;
}
