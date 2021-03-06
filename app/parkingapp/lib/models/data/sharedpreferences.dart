import 'package:shared_preferences/shared_preferences.dart';

class SharedPreferencesHelper {
  //setter for passcode to unlock app
  static Future<bool> setPasscode(String passcode) async {
    SharedPreferences localStorage = await SharedPreferences.getInstance();
    return localStorage.setString('passcode', passcode);
  }

  //getter for passcode to unlock app
  static Future<String> getPasscode() async {
    SharedPreferences localStorage = await SharedPreferences.getInstance();
    return localStorage.getString('passcode');
  }
}
