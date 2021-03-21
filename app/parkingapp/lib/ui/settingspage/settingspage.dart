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
  bool pushNotifications = false;
  bool pushParkIn;

  SharedPreferences preferences;

  @override
  void initState() {
    // TODO: implement initState
    super.initState();
    setState(() {
      SharedPreferences.getInstance().then((SharedPreferences prefs) {
        preferences = prefs;
        this.pushNotifications = prefs.getBool('notifications');
      }).whenComplete(() {
        if (this.pushNotifications == null) {
          this.pushNotifications = true;
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
            value: this.pushNotifications,
            onChanged: (value) {
              if (value) {
                SharedPreferencesHelper.enableNotifications();
              } else {
                SharedPreferencesHelper.disableNotifications();
              }
              setState(() {
                this.pushNotifications = value;
              });
            }),
        Divider(),
        SwitchListTile(
            title: Text('Push Nachrichten II'),
            value: this.pushNotifications,
            onChanged: (value) {
              setState(() {
                this.pushParkIn = value;
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
            }),
        Divider(),
        ListTile(
          title: Text('Push Notifications'),
          onTap: () {
            AppSettings.openNotificationSettings();
          },
        )
      ],
    );
  }

  _passCodeSettings() {
    Navigator.push(context,
        MaterialPageRoute(builder: (BuildContext context) => PasscodePage()));
  }

  _getNotifications() async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    this.pushNotifications = prefs.getBool('authentification') ?? false;
  }

  void setNotifications(bool value) {
    if (value) {
      SharedPreferencesHelper.enableAuthentification();
    } else {
      SharedPreferencesHelper.disableAuthentification();
    }
  }
}
