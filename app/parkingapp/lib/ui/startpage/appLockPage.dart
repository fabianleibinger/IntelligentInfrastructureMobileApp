import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:passcode_screen/circle.dart';
import 'package:passcode_screen/keyboard.dart';
import 'package:passcode_screen/passcode_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';

// Class is only called from [AuthentificationHandling] when app is locked
class AppLockPage extends StatefulWidget {
  final String apikey;
  const AppLockPage({Key key, this.apikey}) : super(key: key);
  @override
  _AppLockPageState createState() => _AppLockPageState();
}

const storedPasscode = '123456';

class _AppLockPageState extends State<AppLockPage> {
  //for passcode screen
  final StreamController<bool> _verificationNotifier =
      StreamController<bool>.broadcast();

  // digits list for passcode screen
  List<String> digits;
  bool isAuthenticated = false;
  @override
  Widget build(BuildContext context) {
    return _passCodeLockScreen(context);
  }

  /// Creates passcode ui
  _passCodeLockScreen(BuildContext context) {
    return PasscodeScreen(
      title: Text(
        'App Passwort eingeben',
        textAlign: TextAlign.center,
        style: TextStyle(color: Colors.white, fontSize: 28),
      ),
      circleUIConfig:
          CircleUIConfig(borderColor: green, fillColor: green, circleSize: 30),
      keyboardUIConfig:
          KeyboardUIConfig(digitBorderWidth: 2, primaryColor: green),
      passwordEnteredCallback: _onPasscodeEntered,
      cancelButton: Text(
        'Delete',
        style: const TextStyle(fontSize: 16, color: Colors.white),
        semanticsLabel: 'Delete',
      ),
      deleteButton: Text(
        'Delete',
        style: const TextStyle(fontSize: 16, color: Colors.white),
        semanticsLabel: 'Delete',
      ),
      shouldTriggerVerification: _verificationNotifier.stream,
      backgroundColor: Colors.black.withOpacity(0.8),
      cancelCallback: _onPassCodeCancelled,
      digits: digits,
      passwordDigits: 6,
    );
  }

  // Checks if entered passcode equals stored passcode
  _onPasscodeEntered(String enteredPasscode) async {
    String passcode = await SharedPreferencesHelper.getPasscode() ?? "123456";
    bool isValid = passcode == enteredPasscode;
    //_verificationNotifier.add(isValid);
    if (isValid) {
      setState(() {
        this.isAuthenticated = isValid;
      });
      // Routes to [RouteLandingPage] after unlock
      Navigator.pushReplacementNamed(context, Routes.routeLandingPage);
      this.isValidCallBack();
    }
  }

  _onPassCodeCancelled() {
    Navigator.maybePop(context);
  }

  @override
  void dispose() {
    _verificationNotifier.close();
    super.dispose();
  }

  void isValidCallBack() {}
}

/// Handels routing if app is locked,
///
/// directing to [AppLockPage] if a passcode is required,
/// directing to [RouteLandingPage] if no passcode is required
class AuthentificationHandling extends StatelessWidget {
  static const String routeName = '/authPage';

  @override
  Widget build(BuildContext context) {
    _isAuthentificated(context);
    return Center(
      child: CircularProgressIndicator(),
    );
  }

  /// Checks if app is secured with passcode and handels routing
  void _isAuthentificated(BuildContext context) async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    bool isAuthenticated = prefs.getBool('authentification') ?? false;
    print(isAuthenticated);
    isAuthenticated
        ? Navigator.push(context,
            MaterialPageRoute(builder: (BuildContext context) => AppLockPage()))
        : Navigator.pushNamedAndRemoveUntil(
            context, Routes.routeLandingPage, (Route<dynamic> route) => false);
  }
}
