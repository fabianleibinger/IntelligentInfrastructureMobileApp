import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_app_lock/flutter_app_lock.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:passcode_screen/circle.dart';
import 'package:passcode_screen/keyboard.dart';
import 'package:passcode_screen/passcode_screen.dart';

class FirstStartPage extends StatefulWidget {
  final String apikey;

  const FirstStartPage({Key key, this.apikey}) : super(key: key);
  @override
  _FirstStartPageState createState() => _FirstStartPageState();
}

const storedPasscode = '123456';

class _FirstStartPageState extends State<FirstStartPage> {
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
    return _passCodeLockScreen();
  }

  _passCodeLockScreen() {
    return PasscodeScreen(
      title: Text(
        'App Passwort eingeben',
        textAlign: TextAlign.center,
        style: TextStyle(color: Colors.white, fontSize: 28),
      ),
      circleUIConfig: CircleUIConfig(
          borderColor: Colors.green, fillColor: Colors.green, circleSize: 30),
      keyboardUIConfig:
          KeyboardUIConfig(digitBorderWidth: 2, primaryColor: Colors.green),
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

  _onPasscodeEntered(String enteredPasscode) {
    bool isValid;

    //for testing without saving passcode before
    SharedPreferencesHelper.setPasscode('123456');
    //AppLock.of(context).didUnlock();
    FutureBuilder<String>(
      future: SharedPreferencesHelper.getPasscode(),
      builder: (BuildContext context, AsyncSnapshot<String> snapshot) {
        String savedPasscode = snapshot.data.toString();
        if (enteredPasscode.compareTo(savedPasscode) == 0) {
          isValid = true;
        } else {
          isValid = false;
        }
        _verificationNotifier.add(isValid);
        if (isValid) {
          setState(() {
            this.isAuthenticated = isValid;
          });
          AppLock.of(context).didUnlock();
        }
        return CircularProgressIndicator();
      },
    );
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
