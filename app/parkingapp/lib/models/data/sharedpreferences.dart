import 'package:shared_preferences/shared_preferences.dart';

class SharedPreferencesHelper {
  static Future<bool> getAllowNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notifications') ?? false;
  }

  static Future<bool> setAllowNotifications(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notifications', value);
  }
}
