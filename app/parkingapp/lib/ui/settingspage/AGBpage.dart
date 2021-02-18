import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';

class AGB extends StatelessWidget {
  static const String routeName = '/agbpage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
          title: Text('AGB und Nutzungsbedingungen', style: whiteHeader),
          leading: IconButton(
            icon: Icon(Icons.arrow_back_ios),
            onPressed: () {
              Navigator.popAndPushNamed(context, SettingsPage.routeName);
            },
          )),
      body: Text('AGB'),
    );
  }
}
