import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
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
      _pushLoad = true,
      _pushPark = true,
      _pushNear = true;
  List<Widget> _messagesOn = [];

  @override
  Widget build(BuildContext context) {
    return Form(
        key: _formKey,
        child: Padding(
          padding: EdgeInsets.all(10),
          child: ListView(
            children: <Widget>[
              SwitchListTile(
                title: Text('Push Nachrichten'),
                subtitle: Text('Hinweise erhalten, bei wichtigen Hinweisen!'),
                onChanged: (bool value) {
                  setState(() {
                    _pushMessages = value;
                    if (_pushMessages) {
                      _messagesOn.addAll([
                        SwitchListTile(
                          title: Text('Ladevorgang'),
                          subtitle: Text(
                              'Hinweise erhalten, bei wichtigen Hinweisen!'),
                          value: _pushLoad,
                          onChanged: (bool value) {
                            setState(() {
                              _pushLoad = value;
                            });
                          },
                        ),
                        Divider(),
                        SwitchListTile(
                          title: Text('Einparkvorgang'),
                          subtitle: Text(
                              'Hinweise erhalten, bei wichtigen Hinweisen!'),
                          value: _pushPark,
                          onChanged: (bool value) {
                            setState(() {
                              _pushPark = value;
                            });
                          },
                        ),
                        Divider(),
                        SwitchListTile(
                          title: Text('In der Nähe des Parkhauses'),
                          subtitle: Text(
                              'Hinweise erhalten, bei wichtigen Hinweisen!'),
                          value: _pushNear,
                          onChanged: (bool value) {
                            setState(() {
                              _pushNear = value;
                            });
                          },
                        ),
                      ]);
                    } else {
                      _messagesOn.clear();
                    }
                  });
                },
                value: _pushMessages,
              ),
              Divider(),
              Column(
                children: _messagesOn,
              ),
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
        ));
  }
}
