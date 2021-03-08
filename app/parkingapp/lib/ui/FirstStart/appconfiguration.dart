import 'package:flutter/material.dart';
import 'package:parkingapp/ui/FirstStart/addvehicle.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/settingspage/changepasscodepage.dart';
import 'package:parkingapp/util/qrscanner.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';

class AppConfiguration extends StatefulWidget {
  @override
  _AppConfigurationState createState() => _AppConfigurationState();
}

class _AppConfigurationState extends State<AppConfiguration> {
  final _formKey = GlobalKey<FormState>();
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).titleConfigureApp),
        ),
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
        body: Center(
            child: Form(
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
                        subtitle:
                            'Push Benachrichtigungen zum Ladevorgang erhalten',
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
                          Navigator.push(
                              context,
                              MaterialPageRoute(
                                  builder: (BuildContext context) =>
                                      PasscodePage()));
                        },
                      ),
                      ListTile(
                          title: Text("Daten importieren"),
                          subtitle: Text(""),
                          trailing: Icon(Icons.arrow_forward_ios),
                          onTap: () {
                            Navigator.push(
                                context,
                                MaterialPageRoute(
                                    builder: (BuildContext context) =>
                                        QRScanner()));
                          })
                    ],
                  ),
                ))));
  }
}
