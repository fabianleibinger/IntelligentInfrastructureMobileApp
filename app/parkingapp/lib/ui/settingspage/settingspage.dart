import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/agbsdialog.dart';
import 'package:parkingapp/dialogs/profileqrdialog.dart';
import 'package:parkingapp/dialogs/vehicleqrdialog.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:settings_ui/settings_ui.dart';

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
      appBar: AppBar(title: Text('Einstellungen', style: whiteHeader)),
      body: ListView(
        children: <Widget>[
          ListTile(
            title: Text('Language'),
            leading: Icon(Icons.language),
            onTap: () {},
          ),
          SettingsTile.switchTile(
            title: 'Use fingerprint',
            leading: Icon(Icons.fingerprint),
            switchValue: true,
            onToggle: (bool value) {},
          ),
        ],
      ),
    );
  }
}
