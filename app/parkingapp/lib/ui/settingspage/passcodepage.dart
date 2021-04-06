import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:passcode_screen/circle.dart';
import 'package:passcode_screen/keyboard.dart';
import 'package:passcode_screen/passcode_screen.dart';

/// Class for the widget to select a passcode
class PasscodePage extends StatefulWidget {
  static const String routeName = '/passcodepage';
  PasscodePage({Key key}) : super(key: key);

  @override
  State<StatefulWidget> createState() => PasscodePageState();
}

class PasscodePageState extends State<PasscodePage> {
  final StreamController<bool> _verificationNotifier =
      StreamController<bool>.broadcast();

  bool isAuthenticated = false;
  List<String> digits;
  @override
  Widget build(BuildContext context) {
    return showPasscodeScreen();
  }

  /// Creates UI for passcode screen
  Widget showPasscodeScreen() {
    return PasscodeScreen(
        title: Text(
          'Neues App Passwort eingeben',
          textAlign: TextAlign.center,
          style: TextStyle(color: Colors.white, fontSize: 28),
        ),
        circleUIConfig: CircleUIConfig(
            borderColor: Theme.of(context).primaryColor,
            fillColor: Theme.of(context).primaryColor,
            circleSize: 30),
        keyboardUIConfig: KeyboardUIConfig(
            digitBorderWidth: 2, primaryColor: Theme.of(context).primaryColor),
        passwordEnteredCallback: _onNewPasscodeEntered,
        cancelButton: Icon(
          Icons.arrow_back,
          color: Theme.of(context).primaryColor,
        ),
        deleteButton: Text(
          'Delete',
          style: const TextStyle(fontSize: 16, color: Colors.white),
          semanticsLabel: 'Delete',
        ),
        bottomWidget: _buildPasscodeRestoreButton(),
        shouldTriggerVerification: _verificationNotifier.stream,
        backgroundColor: Colors.black.withOpacity(0.8),
        cancelCallback: _onPasscodeCancelled,
        digits: digits,
        passwordDigits: 6);
  }

  /// Handels new passcode
  _onNewPasscodeEntered(String enteredPasscode) {
    bool isValid;
    //passcode screen disappears after typing in the passcode
    if (enteredPasscode.length == 6) {
      isValid = true;
      SharedPreferencesHelper.setPasscode(enteredPasscode);
    } else {
      isValid = false;
    }
    _verificationNotifier.add(isValid);
    if (isValid) {
      setState(() {
        this.isAuthenticated = isValid;
      });
      SharedPreferencesHelper.enableAuthentification();
    }
  }

  /// Method to pop the widget
  _onPasscodeCancelled() {
    Navigator.maybePop(context);
  }

  /// Method to build restore button
  _buildPasscodeRestoreButton() => Align(
        alignment: Alignment.bottomCenter,
        child: Container(
          margin: const EdgeInsets.only(bottom: 10.0, top: 20.0),
          child: FlatButton(
            child: Text(
              "App Passwort ausschalten",
              textAlign: TextAlign.center,
              style: const TextStyle(
                  fontSize: 16,
                  color: Colors.white,
                  fontWeight: FontWeight.w300),
            ),
            splashColor: Colors.white.withOpacity(0.4),
            highlightColor: Colors.white.withOpacity(0.2),
            onPressed: () {
              _resetAppPassword();
              Navigator.maybePop(context);
            },
          ),
        ),
      );

  /// Resets passcode in shared preferences
  _resetAppPassword() {
    SharedPreferencesHelper.disableAuthentification();
  }

  @override
  void dispose() {
    _verificationNotifier.close();
    super.dispose();
  }
}
