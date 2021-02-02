import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/firststartpage/firststartpage.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehicleblocobserver.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/provider.dart';

// Main: From here you call all u'r widgets.

void main() {
  Bloc.observer = VehicleBlocObserver();
  runApp(Main());
}

class Main extends StatelessWidget {
  //defines MaterialApp used by this program. [homeWidget] is the home child of MaterialApp
  static MaterialApp getMaterialApp(Widget homeWidget) {
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
      debugShowCheckedModeBanner: false,
      onGenerateTitle: (BuildContext context) =>
          AppLocalizations.of(context).appTitle,
      theme: themeData,
      home: homeWidget,
      routes: {
        Routes.main: (context) => MainPage(),
        Routes.settings: (context) => SettingsPage(),
        Routes.vehicle: (context) => VehiclePage(),
      },
    );
  }

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    //TODO move ListenableProvider into getMaterialApp method. For some reason ListenableProvider is not initialized if built in getMaterialApp
    return ListenableProvider(
      child: getMaterialApp(MyHomePage()), //Provider for Drawer
      create: (_) => DrawerStateInfo(),
    );
  }
}

class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return MainPage();
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
}
