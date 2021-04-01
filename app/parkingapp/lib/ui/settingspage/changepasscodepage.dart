import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:passcode_screen/circle.dart';
import 'package:passcode_screen/keyboard.dart';
import 'package:passcode_screen/passcode_screen.dart';

const storedPasscode = '123456';

class PasscodePage extends StatefulWidget {
  static const String routeName = '/changepasscodepage';
  final VoidCallback login;
  PasscodePage({Key key, this.login}) : super(key: key);

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
        passwordEnteredCallback: onNewPasscodeEntered,
        cancelButton: Icon(
          Icons.arrow_back,
          color: Theme.of(context).primaryColor,
        ),
        deleteButton: Text(
          'Delete',
          style: const TextStyle(fontSize: 16, color: Colors.white),
          semanticsLabel: 'Delete',
        ),
        bottomWidget: buildPasscodeRestoreButton(),
        shouldTriggerVerification: _verificationNotifier.stream,
        backgroundColor: Colors.black.withOpacity(0.8),
        cancelCallback: onPasscodeCancelled,
        digits: digits,
        passwordDigits: 6);
  }

  onNewPasscodeEntered(String enteredPasscode) {
    bool isValid;
    //damit das Eingabefenster wieder verschwindet
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

  onPasscodeCancelled() {
    Navigator.maybePop(context);
  }

  buildPasscodeRestoreButton() => Align(
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
              resetAppPassword();
              Navigator.maybePop(context);
            },
          ),
        ),
      );

  resetAppPassword() {
    SharedPreferencesHelper.disableAuthentification();
  }

  @override
  void dispose() {
    _verificationNotifier.close();
    super.dispose();
  }
}
