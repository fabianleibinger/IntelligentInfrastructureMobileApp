import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
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
