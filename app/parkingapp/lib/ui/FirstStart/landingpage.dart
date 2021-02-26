import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/FirstStart/termsofservice.dart';
import 'package:shared_preferences/shared_preferences.dart';

class LandingPage extends StatelessWidget {
  static const String routeName = '/LandingPage';

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

class RouteLandingPage extends StatelessWidget {
  static const String routeName = '/RouteLandingPage';

  @override
  Widget build(BuildContext context) {
    _isSetUp(context);
    return Center(
      child: CircularProgressIndicator(),
    );
  }

  void _isSetUp(BuildContext context) async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    bool isSetUp = prefs.getBool('isSetUp') ?? false;
    print('isSetUp? ' + isSetUp.toString());
    //navigate to page
    isSetUp
        ? Navigator.pushReplacementNamed(context, Routes.vehicle)
        : Navigator.pushReplacementNamed(context, Routes.landingPage);
  }
}
