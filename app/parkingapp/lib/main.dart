import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/coordinate.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/FirstStart/landingpage.dart';
import 'package:parkingapp/ui/FirstStart/routelandingpage.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/ui/parkpages/parkinpage.dart';
import 'package:parkingapp/ui/parkpages/parkoutpage.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/settingspage/transferkeys.dart';
import 'package:parkingapp/ui/startpage/appLockPage.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';
import 'package:parkingapp/util/qrscanner.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/provider.dart';
import 'models/classes/parkinggarage.dart';
import 'models/enum/parkinggaragetype.dart';

// Main: From here you call all ur widgets.

void main() {
  Provider.debugCheckInvalidValueType = null;
  runApp(Main());
}

class Main extends StatelessWidget {
  //defines MaterialApp used by this program. [homeWidget] is the home child of MaterialApp
  static MaterialApp getMaterialApp(String initialRoute) {
    return MaterialApp(
      //Initialize Localization
      localizationsDelegates: [
        GlobalMaterialLocalizations.delegate,
        GlobalWidgetsLocalizations.delegate,
        GlobalCupertinoLocalizations.delegate,
        AppLocalizations.delegate,
      ],
      supportedLocales: AppLocalizations.supportedLocales,
      // App info
      onGenerateTitle: (BuildContext context) =>
          AppLocalizations.of(context).appTitle,
      theme: themeData,
      darkTheme: darkThemeData,
      initialRoute: initialRoute,
      //Routing of app
      onGenerateRoute: (settings) {
        //basic routes
        switch (settings.name) {
          case Routes.vehicle:
            return MaterialPageRoute(builder: (context) => VehiclePage());
          case Routes.settings:
            return MaterialPageRoute(builder: (context) => SettingsPage());
          case Routes.agbPage:
            return MaterialPageRoute(builder: (context) => AGB());
          case Routes.createVehicle:
            return MaterialPageRoute(builder: (context) => CreateVehicle());
          case Routes.landingPage:
            return MaterialPageRoute(builder: (context) => LandingPage());
          case Routes.routeLandingPage:
            return MaterialPageRoute(builder: (context) => RouteLandingPage());
          case Routes.authPage:
            return MaterialPageRoute(
                builder: (context) => AuthentificationHandling());
          case Routes.qrScanner:
            return MaterialPageRoute(builder: (context) => ScanScreen());
          case Routes.transferKeys:
            return MaterialPageRoute(builder: (context) => Transferkeys());
        }
        //vehicles park routes
        //regex inAppKey check: 80996360-679b-11eb-8046-434ac6c775f0
        RegExp inAppKeyRegExp = RegExp(r'\w{8}-\w{4}-\w{4}-\w{4}-\w{12}');
        var uri = Uri.parse(settings.name);
        if (uri.pathSegments.length > 0 &&
            inAppKeyRegExp.hasMatch(uri.pathSegments.first)) {
          print('vehicle: ' + uri.pathSegments.first);
          //TODO generate vehicle Page with inAppKey
          if (uri.pathSegments.length == 2) {
            //does only need the path behind /
            if (uri.pathSegments.last == Routes.parkOut.split('/').last) {
              //parkOut route
              return MaterialPageRoute(
                  builder: (context) => ParkOutPage(uri.pathSegments.first));
            } else {
              //parkIn route
              return MaterialPageRoute(
                  builder: (context) => ParkInPage(uri.pathSegments.first));
            }
          } else {
            //mainPage route
            return MaterialPageRoute(
                builder: (context) => MainPage(uri.pathSegments.first));
          }
        }

        //fallback route
        return MaterialPageRoute(builder: (context) => SettingsPage());
      },
    );
  }

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    currentParkingGarage = ParkingGarage(
        name: 'Parkgarage Fasanengarten',
        type: ParkingGarageType.Tiefgarage,
        freeParkingSpots: 0,
        freeChargeableParkingSpots: 0,
        image: 'assets/parkgarage-fasanengarten.jpg',
        map: "assets/parkgarage-fasanengarten-map.jpg",
        bottomLeft: Coordinate(latitude: 49.0134227, longitude: 8.41950527853),
        topRight:
            Coordinate(latitude: 49.0144759205, longitude: 8.42059599234));
    //TODO move ListenableProvider into getMaterialApp method. For some reason ListenableProvider is not initialized if built in getMaterialApp
    return MultiProvider(
      providers: [
        BlocProvider<VehicleBloc>(create: (context) {
          return VehicleBloc(<Vehicle>[]);
        }),
        ListenableProvider(
          create: (_) => DrawerStateInfo(Routes.vehicle),
        )
      ],
      child: getMaterialApp(Routes.authPage),
    );
  }
}
