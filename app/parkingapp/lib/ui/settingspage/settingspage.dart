import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/scanqrdialog.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/util/qrscanner.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:app_settings/app_settings.dart';

import 'changepasscodepage.dart';

class SettingsPage extends StatelessWidget {
  static const String routeName = '/settingspage';
  const SettingsPage({Key key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      appBar: AppBar(title: Text('Einstellungen', style: whiteHeader)),
      body: SettingsForm(),
    );
  }
}

class SettingsForm extends StatefulWidget {
  @override
  State<StatefulWidget> createState() => _SettingsFormState();
}

class _SettingsFormState extends State<SettingsForm> {
  bool _pushNotifications = false;
  bool _pushParkIn = false;
  bool _push3 = false;

  SharedPreferences preferences;

  @override
  void initState() {
    super.initState();
    setState(() {
      SharedPreferences.getInstance().then((SharedPreferences prefs) {
        preferences = prefs;
        this._pushNotifications = prefs.getBool('notifications');
      }).whenComplete(() {
        if (this._pushNotifications == null) {
          this._pushNotifications = true;
        }
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return ListView(
      children: <Widget>[
        SwitchListTile(
            title: Text('Push Nachrichten I'),
            value: _pushNotifications,
            onChanged: (value) {
              if (value) {
                AppSettings.openNotificationSettings();
              }
              if (value) {
                SharedPreferencesHelper.enableNotifications();
              } else {
                SharedPreferencesHelper.disableNotifications();
              }
              setState(() {
                this._pushNotifications = value;
              });
            }),
        Divider(),
        SwitchListTile(
            title: Text('Push Nachrichten II'),
            value: _pushParkIn && _pushNotifications,
            onChanged: (value) {
              setState(() {
                this._pushParkIn = value;
              });
            }),
        Divider(),
        SwitchListTile(
            title: Text('Push Nachrichten III'),
            value: _push3 && _pushNotifications,
            onChanged: (value) {
              setState(() {
                _push3 = value;
              });
            }),
        Divider(),
        ListTile(
          title: Text('Passwort'),
          subtitle: Text('App mit einem Passwort schützen'),
          trailing: Icon(Icons.arrow_forward_ios),
          onTap: () {
            _passCodeSettings();
          },
        ),
        Divider(),
        ListTile(
            title: Text('Daten übertragen'),
            subtitle: Text('Fahrezeuge auf andere Geräte übertragen'),
            trailing: Icon(Icons.arrow_forward_ios),
            onTap: () {
              Navigator.pushNamed(context, Routes.transferkeys);
            }),
        Divider(),
        ListTile(
            title: Text('AGB und Nutzungsbedingungen'),
            subtitle: Text('AGB und Nutzungsbedingungen anzeigen'),
            trailing: Icon(Icons.arrow_forward_ios),
            onTap: () {
              Navigator.pushNamed(context, Routes.agbPage);
            })
      ],
    );
  }

  _passCodeSettings() {
    Navigator.push(context,
        MaterialPageRoute(builder: (BuildContext context) => PasscodePage()));
  }

  _getNotifications() async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    bool notifications = prefs.getBool('authentification') ?? false;
    return notifications ? true : false;
  }

  void setNotifications(bool value) {
    if (value) {
      SharedPreferencesHelper.enableAuthentification();
    } else {
      SharedPreferencesHelper.disableAuthentification();
    }
  }
}
