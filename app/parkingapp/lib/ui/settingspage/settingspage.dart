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
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

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
  bool _pushNotificationsParked = false;
  bool _pushNotificationsCharge = false;

  SharedPreferences preferences;

  @override
  void initState() {
    super.initState();
    /*setState(() {
      SharedPreferencesHelper.getNotifications().then((value) {
        _pushNotifications = value;
      }).whenComplete(() {
        //set value if it was not set before
        if (_pushNotifications == null) {
          _pushNotifications = true;
        }
      });
      SharedPreferencesHelper.getNotificationsCharged()().then((value) {
        _pushNotificationsCharge = value;
      }).whenComplete(() {
        //set value if it was not set before
        if (_pushNotificationsCharge == null) {
          _pushNotificationsCharge = true;
        }
      });
      SharedPreferencesHelper.getNotificationsParked()().then((value) {
        _pushNotificationsParked = value;
      }).whenComplete(() {
        //set value if it was not set before
        if (_pushNotificationsParked == null) {
          _pushNotificationsParked = true;
        }
      });
    });*/
  }

  @override
  Widget build(BuildContext context) {
    return ListView(
      children: <Widget>[
        SwitchListTile(
            title: Text(AppLocalizations.of(context).pushMessages),
            subtitle: Text(AppLocalizations.of(context).pushMessagesText),
            value: _pushNotifications,
            onChanged: (value) {
              if (value) {
                AppSettings.openNotificationSettings();
              }
              //update change in Sharedpreferences
              if (value) {
                SharedPreferencesHelper.enableNotifications();
              } else {
                SharedPreferencesHelper.disableNotifications();
              }
              setState(() {
                _pushNotifications = value;
              });
            }),
        Divider(),
        SwitchListTile(
            title: Text(AppLocalizations.of(context).pushMessagesParked),
            subtitle: Text(AppLocalizations.of(context).pushMessagesParkedText),
            value: _pushNotificationsParked && _pushNotifications,
            onChanged: (value) {
              setState(() {
                _pushNotificationsParked = value;
              });
            }),
        Divider(),
        SwitchListTile(
            title: Text(AppLocalizations.of(context).pushMessagesCharge),
            subtitle: Text(AppLocalizations.of(context).pushMessagesChargeText),
            value: _pushNotificationsCharge && _pushNotifications,
            onChanged: (value) {
              setState(() {
                _pushNotificationsCharge = value;
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
}
