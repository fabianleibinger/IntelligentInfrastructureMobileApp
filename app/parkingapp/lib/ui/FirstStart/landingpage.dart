import 'package:flutter/material.dart';
import 'package:parkingapp/ui/FirstStart/termsofservice.dart';

class LandingPage extends StatelessWidget {
  static final String routeName = '/LandingPage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text('Herzlich Willkommen'),
        ),
        bottomNavigationBar: Material(
          elevation: 10,
          child: ButtonBar(
            children: [
              FlatButton(
                child: Text('Weiter'),
                onPressed: () => Navigator.push(context,
                    MaterialPageRoute(builder: (context) => TermsOfService())),
              )
            ],
          ),
        ),
        body: ListView(
          children: [
            Center(
              child: Column(
                children: [Icon(Icons.directions_car), Text('FZI AutoPark')],
              ),
            ),
            Text(
                'Herzlich Willkommen in der FZI Autopark App.\nIn den nächsten Schritten werden Dir die grundlegenden Funktionen der Anwendung näher gebracht und das erste Fahrzeug eingerichtet.'),
          ],
        ));
  }
}
