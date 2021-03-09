import 'package:shared_preferences/shared_preferences.dart';

class SharedPreferencesHelper {
  static Future<bool> setPasscode(String passcode) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setString('passcode', passcode);
  }

  static Future<String> getPasscode() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString('passcode');
  }
}
