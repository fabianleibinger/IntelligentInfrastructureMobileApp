import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/ui/FirstStart/landingpage.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/firststartpage/appLockPage.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/ui/parkpages/parkinpage.dart';
import 'package:parkingapp/ui/parkpages/parkoutpage.dart';
import 'package:parkingapp/ui/settingspage/qrpage.dart';
import 'package:parkingapp/ui/settingspage/settings.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/settingspage/transferkeys.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';

class Routes {
  static const String vehicle = VehiclePage.routeName;
  static const String settings = SettingsPage.routeName;
  static const String agbPage = AGB.routeName;
  static const String parkIn = ParkInPage.routeName;
  static const String parkOut = ParkOutPage.routeName;
  static const String createVehicle = CreateVehicle.routeName;
  static const String landingPage = LandingPage.routeName;
  static const String routeLandingPage = RouteLandingPage.routeName;
  static const String transferkeys = Transferkeys.routeName;
  static const String qrpage = QRPage.routeName;
  static const String authPage = AuthentificationHandling.routeName;

  //returns the correct route for the AppDrawer according to vehicles values.
  //returns either MainPage, ParkInPage, ParkOutPage
  static String returnCorrectRouteForVehicle(Vehicle vehicle) {
    //vehicle is currently parking in
    if (vehicle.parkingIn) {
      print('AppDrawer chose park in page');
      return vehicle.inAppKey + parkIn;
      //vehicle is currently parking out or cancelling park in
    } else if (vehicle.parkingOut) {
      print('AppDrawer chose park out page');
      return vehicle.inAppKey + parkOut;
      //vehicle is parked in
    } else if (vehicle.parkedIn) {
      print('AppDrawer chose park in page');
      return vehicle.inAppKey + parkIn;
      //vehicle is not inside the parking garage
    } else {
      print('AppDrawer chose main page');
      //goto mainPage
      return vehicle.inAppKey;
    }
  }
}
