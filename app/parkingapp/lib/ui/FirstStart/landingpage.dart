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
  static final String isSetUp = 'isSetUp';

  @override
  Widget build(BuildContext context) {
    _isSetUp(context);
    return Center(
      child: CircularProgressIndicator(),
    );
  }

  void _isSetUp(BuildContext context) async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    bool isSetUp = prefs.getBool(RouteLandingPage.isSetUp) ?? false;
    print('isSetUp? ' + isSetUp.toString());
    //navigate to page and remove all widgets from the widget tree
    isSetUp
        ? Navigator.pushNamedAndRemoveUntil(
            context, Routes.vehicle, (Route<dynamic> route) => false)
        : Navigator.pushNamedAndRemoveUntil(
            context, Routes.landingPage, (Route<dynamic> route) => false);
  }
}
