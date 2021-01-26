import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/scanqrdialog.dart';
import 'package:parkingapp/dialogs/drivesourcedialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
// example for a page (mainpage)

class VehiclePage extends StatefulWidget {
  static const String routeName = '/vehiclepage';
  const VehiclePage({Key key}) : super(key: key);
  @override
  _VehiclePageState createState() => _VehiclePageState();
}

class _VehiclePageState extends State<VehiclePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      body: Text("Hello"),
    );
  }
}
