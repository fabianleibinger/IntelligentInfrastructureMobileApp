import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/scanqrdialog.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/util/qrscanner.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:app_settings/app_settings.dart';

import 'changepasscodepage.dart';

class SettingsPage extends StatelessWidget {
  static const String routeName = '/settingspage';
  final String apikey;
  const SettingsPage({Key key, this.apikey}) : super(key: key);

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
  //global key for form validation
  final _formKey = GlobalKey<FormState>();

  bool _pushNotifications;

  @override
  Widget build(BuildContext context) {
    return Form(
        key: _formKey,
        child: Padding(
          padding: EdgeInsets.all(10),
          child: ListView(
            children: <Widget>[
              Padding(padding: EdgeInsets.symmetric(vertical: 0.5)),
              SwitchSettingsTile(
                settingKey: 'pushNotifications',
                title: 'Push Nachrichten',
                subtitle: 'Hinweise erhalten, bei wichtigen Hinweisen',
              ),
              SwitchSettingsTile(
                settingKey: 'pushLoad',
                title: 'Ladevorgang',
                subtitle: 'Push Benachrichtigungen zum Ladevorgang erhalten',
                visibleIfKey: 'pushNotifications',
              ),
              SwitchSettingsTile(
                settingKey: 'pushPark',
                title: 'Einparkvorgang',
                subtitle:
                    'Push Benachrichtigungen bei abgeschlossenem Einparkvorgang',
                visibleIfKey: 'pushNotifications',
              ),
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
          ),
        ));
  }

  _passCodeSettings() {
    Navigator.push(context,
        MaterialPageRoute(builder: (BuildContext context) => PasscodePage()));
  }
}
