import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:passcode_screen/circle.dart';
import 'package:passcode_screen/keyboard.dart';
import 'package:passcode_screen/passcode_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

// Class is only called from [AuthentificationHandling] when app is locked
class AppLockPage extends StatefulWidget {
  final String apikey;
  const AppLockPage({Key key, this.apikey}) : super(key: key);
  @override
  _AppLockPageState createState() => _AppLockPageState();
}

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
        AppLocalizations.of(context).enterPasscode,
        textAlign: TextAlign.center,
        style: TextStyle(color: Colors.white, fontSize: 28),
      ),
      circleUIConfig:
          CircleUIConfig(borderColor: green, fillColor: green, circleSize: 30),
      keyboardUIConfig:
          KeyboardUIConfig(digitBorderWidth: 2, primaryColor: green),
      passwordEnteredCallback: _onPasscodeEntered,
      cancelButton: Text(
        AppLocalizations.of(context).deleteButton,
        style: const TextStyle(fontSize: 16, color: Colors.white),
        semanticsLabel: AppLocalizations.of(context).deleteButton,
      ),
      deleteButton: Text(
        AppLocalizations.of(context).deleteButton,
        style: const TextStyle(fontSize: 16, color: Colors.white),
        semanticsLabel: AppLocalizations.of(context).deleteButton,
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
    if (isValid) {
      setState(() {
        this.isAuthenticated = isValid;
      });
      // Routes to [RouteLandingPage] after unlock
      Navigator.pushReplacementNamed(context, Routes.routeLandingPage);
    } else {
      // Reloads page with empty passcode field
      Navigator.pop(context);
      Navigator.pushNamed(context, Routes.authPage);
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
    print("Passcode required: " + isAuthenticated.toString());
    isAuthenticated
        ? Navigator.push(context,
            MaterialPageRoute(builder: (BuildContext context) => AppLockPage()))
        : Navigator.pushNamedAndRemoveUntil(
            context, Routes.routeLandingPage, (Route<dynamic> route) => false);
  }
}
