import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// This will show a circular progress indicator while it waits for the [SharedPreferences] to be checked if the app has been set up successfully
class RouteLandingPage extends StatelessWidget {
  static const String routeName = '/RouteLandingPage';
  static final String isSetUp = 'isSetUp';

  @override
  Widget build(BuildContext context) {
    DataHelper.initVehicles(context);
    _isSetUp(context);
    return Container(
      color: Theme.of(context).primaryColor,
      child: Center(
        child: CircularProgressIndicator(),
      ),
    );
  }

  /// Check the [SharedPreferences] and database if the app has been successfully set up and navigate to correct page
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
