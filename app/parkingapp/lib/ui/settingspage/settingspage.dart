import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/agbsdialog.dart';
import 'package:parkingapp/dialogs/profileqrdialog.dart';
import 'package:parkingapp/dialogs/vehicleqrdialog.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:settings_ui/settings_ui.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';

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

  bool _pushMessages = false,
      _pushLoad = false,
      _pushPark = false,
      _pushNear = false;
  List<Widget> _messagesOn = [];

  @override
  Widget build(BuildContext context) {
    return Form(
        key: _formKey,
        child: Padding(
          padding: EdgeInsets.all(10),
          child: ListView(
            children: <Widget>[
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
                  Navigator.pushReplacementNamed(context, AGB.routeName);
                },
              ),
              Divider(),
              ListTile(
                  title: Text('Daten übertragen'),
                  subtitle: Text('Fahrezeuge auf andere Geräte übertragen'),
                  trailing: Icon(Icons.arrow_forward_ios),
                  onTap: () {
                    Navigator.pushReplacementNamed(context, AGB.routeName);
                  }),
              Divider(),
              ListTile(
                  title: Text('AGB und Nutzungsbedingungen'),
                  subtitle: Text('AGB und Nutzungsbedingungen anzeigen lassen'),
                  trailing: Icon(Icons.arrow_forward_ios),
                  onTap: () {
                    Navigator.pushReplacementNamed(context, AGB.routeName);
                  })
            ],
          ),
        ));
  }
}
