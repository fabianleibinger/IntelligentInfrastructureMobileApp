import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

class Settings extends StatefulWidget {
  static const String routeName = '/settingspage';
  @override
  State<StatefulWidget> createState() {
    State<StatefulWidget> createState() => _SettingsState();
  }
}

class _SettingsState extends State<Settings> {
  bool _push = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      appBar: AppBar(title: Text('Einstellungen', style: whiteHeader)),
      body: ListView(
        children: [
          SwitchListTile(
              value: _push,
              onChanged: (bool newValue) {
                setState(() {
                  _push = newValue;
                });
              })
        ],
      ),
    );
  }
}
