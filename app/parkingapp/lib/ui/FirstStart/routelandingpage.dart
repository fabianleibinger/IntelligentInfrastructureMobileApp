import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/startpage/appLockPage.dart';
import 'package:shared_preferences/shared_preferences.dart';

//this will show a circular progress indicator while it waits for the shared preferences to be checked
class RouteLandingPage extends StatelessWidget {
  static const String routeName = '/RouteLandingPage';
  static final String isSetUp = 'isSetUp';

  @override
  Widget build(BuildContext context) {
    _isSetUp(context);
    return Container(
      color: Theme.of(context).primaryColor,
      child: Center(
        child: CircularProgressIndicator(),
      ),
    );
  }

  //check the shared preferences and database if the app has been succesfully set up and navigate to correct page
  void _isSetUp(BuildContext context) async {
    await DataHelper.initVehicles(context);
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
