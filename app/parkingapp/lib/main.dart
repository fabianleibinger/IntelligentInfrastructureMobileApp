import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_app_lock/flutter_app_lock.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/FirstStart/landingpage.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/firststartpage/firststartpage.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/ui/settingspage/changepasscodepage.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/provider.dart';

// Main: From here you call all u'r widgets.

void main() {
  Provider.debugCheckInvalidValueType = null;
  runApp(AppLock(
    builder: (args) => Main(),
    lockScreen: FirstStartPage(),
    enabled: false,
  ));
}

class Main extends StatelessWidget {
  //defines MaterialApp used by this program. [homeWidget] is the home child of MaterialApp
  static MaterialApp getMaterialApp(String initialroute) {
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
      initialRoute: initialroute,
      //Routing of app
      onGenerateRoute: (settings) {
        //settings Route
        if (settings.name == Routes.settings) {
          return MaterialPageRoute(builder: (context) => SettingsPage());
        }
        //edit vehicles route
        if (settings.name == Routes.vehicle) {
          return MaterialPageRoute(builder: (context) => VehiclePage());
        }
        //vehicles park routes
        //regex inAppKey check: 80996360-679b-11eb-8046-434ac6c775f0
        RegExp inAppKeyRegExp = RegExp(r'\w{8}-\w{4}-\w{4}-\w{4}-\w{12}');
        var uri = Uri.parse(settings.name);
        if (uri.pathSegments.length > 0 &&
            inAppKeyRegExp.hasMatch(uri.pathSegments.first)) {
          print('vehicle: ' + uri.pathSegments.first);
          //TODO generate vehicle Page with inAppKey
          return MaterialPageRoute(
              builder: (context) => MainPage(uri.pathSegments.first));
        }
        //editVehicle route
        if (settings.name == Routes.createVehicle) {
          return MaterialPageRoute(builder: (context) => CreateVehicle());
        }
        //first start route
        if (settings.name == Routes.landingPage) {
          return MaterialPageRoute(builder: (context) => LandingPage());
        }
        if (settings.name == Routes.routeLandingPage) {
          return MaterialPageRoute(builder: (context) => RouteLandingPage());
        }
        if (settings.name == Routes.agbPage) {
          return MaterialPageRoute(builder: (context) => AGB());
        }
        //fallback route
        return MaterialPageRoute(builder: (context) => SettingsPage());
      },
    );
  }

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
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
      child: getMaterialApp(Routes.routeLandingPage),
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
