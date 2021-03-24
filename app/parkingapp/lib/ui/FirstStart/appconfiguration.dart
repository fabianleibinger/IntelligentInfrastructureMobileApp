import 'package:app_settings/app_settings.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/notifications/notifications.dart';
import 'package:parkingapp/ui/FirstStart/addvehicle.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/settingspage/changepasscodepage.dart';
import 'package:parkingapp/util/qrscanner.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';
import 'package:system_settings/system_settings.dart';

class AppConfiguration extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      appBar: AppBar(
          title: Text(AppLocalizations.of(context).titleConfigureApp,
              style: whiteHeader)),
      bottomNavigationBar: Material(
        elevation: 10,
        child: ButtonBar(
          children: [
            FlatButton(
              child: Text(AppLocalizations.of(context).buttonContinue),
              onPressed: () => Navigator.push(context,
                  MaterialPageRoute(builder: (context) => AddVehicle())),
            )
          ],
        ),
      ),
      body: AppConfigurationForm(),
    );
  }
}

class AppConfigurationForm extends StatefulWidget {
  @override
  _AppConfigurationState createState() => _AppConfigurationState();
}

class _AppConfigurationState extends State<AppConfigurationForm> {
  bool _pushNotifications = false;
  bool _pushNotificationsParked = false;
  bool _pushNotificationsCharge = false;

  Future<SharedPreferences> preferences = SharedPreferences.getInstance();

  @override
  void initState() {
    super.initState();
    getPushNotifications();
  }

  @override
  Widget build(BuildContext context) {
    return ListView(
      children: <Widget>[
        SwitchListTile(
            title: Text(AppLocalizations.of(context).pushMessages),
            subtitle: Text(AppLocalizations.of(context).pushMessagesText),
            value: _pushNotifications,
            onChanged: (value) async {
              if (value) {
                if (await Permission.notification.request().isDenied) {
                  SystemSettings.app();
                }
                if (await Permission.notification.request().isDenied) {
                  value = false;
                }
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
              SharedPreferencesHelper.setNotificationsParked(value);
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
              SharedPreferencesHelper.setNotificationsCharged(value);
              setState(() {
                _pushNotificationsCharge = value;
              });
            }),
        Divider(),
        ListTile(
          title: Text(AppLocalizations.of(context).password),
          subtitle: Text(AppLocalizations.of(context).passwordSubtitle),
          trailing: Icon(Icons.arrow_forward_ios),
          onTap: () {
            _passCodeSettings();
          },
        )
      ],
    );
  }

  _passCodeSettings() {
    Navigator.push(context,
        MaterialPageRoute(builder: (BuildContext context) => PasscodePage()));
  }

  //get all the shared prefernces for initstate
  Future<Null> getPushNotifications() async {
    //inialize SharedPreferences Settings with false values
    SharedPreferencesHelper.initializeSharedPreferences();

    setState(() {
      _pushNotifications = false;
      _pushNotificationsCharge = false;
      _pushNotificationsParked = false;
    });
  }
}
