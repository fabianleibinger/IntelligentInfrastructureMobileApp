import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:passcode_screen/circle.dart';
import 'package:passcode_screen/keyboard.dart';
import 'package:passcode_screen/passcode_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';

class AppLockPage extends StatefulWidget {
  final String apikey;
  const AppLockPage({Key key, this.apikey}) : super(key: key);
  @override
  _AppLockPageState createState() => _AppLockPageState();
}

const storedPasscode = '123456';

class _AppLockPageState extends State<AppLockPage> {
  //nötig??
  //TextEditingController usernameController = new TextEditingController();

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
      cancelCallback: _onPasscodeCancelled,
      digits: digits,
      passwordDigits: 6,
    );
  }

  _onPasscodeEntered(String enteredPasscode) async {
    String passcode = await SharedPreferencesHelper.getPasscode() ?? "123456";
    bool isValid = passcode == enteredPasscode;
    _verificationNotifier.add(isValid);
    if (isValid) {
      setState(() {
        this.isAuthenticated = isValid;
        Navigator.pushReplacementNamed(context, Routes.settings);
      });
    }
  }

  _onPasscodeCancelled() {
    Navigator.maybePop(context);
  }

  @override
  void dispose() {
    _verificationNotifier.close();
    super.dispose();
  }
}

class AuthentificationHandling extends StatelessWidget {
  static const String routeName = '/authPage';

  @override
  Widget build(BuildContext context) {
    _isAuthentificated(context);
    return Center(
      child: CircularProgressIndicator(),
    );
  }

  void _isAuthentificated(BuildContext context) async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    bool isAuthentificated = prefs.getBool('authentification') ?? false;
    isAuthentificated
        ? Navigator.push(context,
            MaterialPageRoute(builder: (BuildContext context) => AppLockPage()))
        : Navigator.pushNamedAndRemoveUntil(
            context, Routes.routeLandingPage, (Route<dynamic> route) => false);
  }
}
