import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/FirstStart/termsofservice.dart';
import 'package:parkingapp/ui/firststartpage/appLockPage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class LandingPage extends StatelessWidget {
  static const String routeName = '/LandingPage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).titleWelcome),
        ),
        bottomNavigationBar: Material(
          elevation: 10,
          child: ButtonBar(
            children: [
              FlatButton(
                child: Text(AppLocalizations.of(context).buttonContinue),
                onPressed: () => Navigator.push(context,
                    MaterialPageRoute(builder: (context) => TermsOfService())),
              )
            ],
          ),
        ),
        body: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Center(
              child: Column(
                children: [
                  Icon(
                    Icons.directions_car,
                    size: 150,
                  ),
                  Text(AppLocalizations.of(context).appName)
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(10.0),
              child: Text(
                AppLocalizations.of(context).welcomeText,
                textAlign: TextAlign.justify,
              ),
            ),
          ],
        ));
  }
}

//this will show a circular progress indicator while it waits for the shared preferences to be checked
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

  //check the shared preferences if the app has been succesfully set up and navigate to correct page
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

  void _isAuthentificated(BuildContext context) async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    bool isAuthentificated = prefs.getBool('authentification') ?? false;
    isAuthentificated
        ? Navigator.push(context,
            MaterialPageRoute(builder: (BuildContext context) => AppLockPage()))
        : Navigator.pushNamedAndRemoveUntil(
            context, Routes.settings, (Route<dynamic> route) => false);
  }
}
