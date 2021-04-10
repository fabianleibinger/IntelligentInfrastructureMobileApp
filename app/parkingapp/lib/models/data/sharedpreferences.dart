import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/notifications/notifications.dart';

/// Handels access to sharedpreferences of app
///
/// Is used for accessing passcode, authentification status handling and
/// notification permissions given by the user
class SharedPreferencesHelper {
  static Future<void> initializeSharedPreferences() async {
    final prefs = await SharedPreferences.getInstance();
    prefs.setBool('authentification', false);
    prefs.setBool('notifications', false);
    prefs.setBool('notificationsParked', false);
    prefs.setBool('notificationsCharged', false);
    Notifications.getEnabledValues();
  }

  static Future<bool> setPasscode(String passcode) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setString('passcode', passcode);
  }

  static Future<String> getPasscode() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString('passcode');
  }

  static Future<bool> enableAuthentification() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('authentification', true);
  }

  static Future<bool> getAuthentificationStatus() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('authentification');
  }

  static Future<bool> disableAuthentification() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('authentification', false);
  }

  static Future<dynamic> enableNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs
        .setBool('notifications', true)
        .then((value) => Notifications.getEnabledValues());
  }

  static Future<dynamic> disableNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs
        .setBool('notifications', false)
        .then((value) => Notifications.getEnabledValues());
  }

  static getNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notifications');
  }

  static setNotificationsParked(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs
        .setBool('notificationsParked', value)
        .then((value) => Notifications.getEnabledValues());
  }

  static setNotificationsCharged(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs
        .setBool('notificationsCharged', value)
        .then((value) => Notifications.getEnabledValues());
  }

  static getNotificationsParked() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notificationsParked');
  }

  static getNotificationsCharged() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notificationsCharged');
  }
}
