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
  //global key for form validation
  final _formKey = GlobalKey<FormState>();
  bool _pushMessages = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        drawer: AppDrawer(),
        appBar: AppBar(title: Text('Einstellungen', style: whiteHeader)),
        body: Form(
            key: _formKey,
            child: Padding(
              padding: EdgeInsets.all(10),
              child: ListView(
                children: <Widget>[
                  SwitchListTile(
                    title: Text('Push Nachrichten'),
                    subtitle:
                        Text('Hinweise erhalten, bei wichtigen Hinweisen'),
                    value: _pushMessages,
                    onChanged: (bool value) {
                      setState(() {
                        _pushMessages = value;
                      });
                    },
                  ),
                  Divider(),
                  ListTile(
                    title: Text('Daten übertragen'),
                    subtitle: Text('Daten auf neues Gerät übertragen'),
                    trailing: Icon(Icons.arrow_forward_ios),
                    onTap: () {},
                  ),
                  Divider(),
                  ListTile(
                    title: Text('Passwort'),
                    subtitle: Text('App mit einem Passwort schützen'),
                    trailing: Icon(Icons.arrow_forward_ios),
                    onTap: () {},
                  ),
                  Divider(),
                  ListTile(
                    title: Text('AGB und Nutzungsbedingungen'),
                    subtitle: Text('Anzeigen der AGB und Nutzungsbedingungen'),
                    trailing: Icon(Icons.arrow_forward_ios),
                    onTap: () {},
                  ),
                  Divider(),
                ],
              ),
            )));
  }
}
