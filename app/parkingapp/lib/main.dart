import 'package:flutter/material.dart';
import 'package:flutter_app_lock/flutter_app_lock.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/FirstStart/landingpage.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/firststartpage/appLockPage.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/ui/parkpages/parkinpage.dart';
import 'package:parkingapp/ui/parkpages/parkoutpage.dart';
import 'package:parkingapp/ui/settingspage/qrpage.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/settingspage/transferkeys.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/provider.dart';
import 'models/classes/parkinggarage.dart';
import 'models/enum/parkinggaragetype.dart';

// Main: From here you call all u'r widgets.

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
        if (settings.name == Routes.agbPage) {
          return MaterialPageRoute(builder: (context) => AGB());
        }
        if (settings.name == Routes.transferkeys) {
          return MaterialPageRoute(builder: (context) => Transferkeys());
        }
        if (settings.name == Routes.qrpage) {
          return MaterialPageRoute(builder: (context) => QRPage());
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
        'Parkgarage Fasanengarten',
        ParkingGarageType.Tiefgarage,
        0,
        0,
        'assets/parkgarage-fasanengarten.jpg');
    //TODO move ListenableProvider into getMaterialApp method. For some reason ListenableProvider is not initialized if built in getMaterialApp
    return MultiProvider(
      providers: [
        BlocProvider<VehicleBloc>(create: (context) {
          return VehicleBloc(List<Vehicle>());
        }),
        ListenableProvider(
          create: (_) => DrawerStateInfo(Routes.vehicle),
        )
      ],
      child: getMaterialApp(Routes.authPage),
    );
  }
}
/*
class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return Text("ijawd");
  }

  void login() {
    setState(() {
      build(context);
    });
  }

  @override
  void initState() {
    super.initState();
  }
}*/
