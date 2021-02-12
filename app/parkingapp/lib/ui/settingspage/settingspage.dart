import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/agbsdialog.dart';
import 'package:parkingapp/dialogs/profileqrdialog.dart';
import 'package:parkingapp/dialogs/vehicleqrdialog.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

class SettingsPage extends StatefulWidget {
  static const String routeName = '/settingspage';
  final String apikey;

  const SettingsPage({Key key, this.apikey}) : super(key: key);
  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      body: Text("Hello"),
    );
  }
}
