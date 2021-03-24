import 'package:shared_preferences/shared_preferences.dart';

class SharedPreferencesHelper {
  static Future<void> initializeSharedPreferences() async {
    final prefs = await SharedPreferences.getInstance();
    prefs.setBool('authentification', false);
    prefs.setBool('notifications', false);
    prefs.setBool('notificationsParked', false);
    prefs.setBool('notificationsCharged', false);
  }

    final prefs = await SharedPreferences.getInstance();
  }

    final prefs = await SharedPreferences.getInstance();
  static Future<bool> enableAuthentification() async {
    return prefs.setBool('authentification', true);
  static Future<bool> getAuthentificationStatus() async {
    return prefs.getBool('authentification');
  }

  static Future<bool> disableAuthentification() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('authentification', false);
  }

  static Future<bool> enableNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notifications', true);
  }

  static Future<bool> disableNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notifications', false);
  }

  static getNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notifications');
  }

  static setNotificationsParked(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notificationsParked', value);
  }

  static setNotificationsCharged(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notificationsCharged', value);
  }

  static getNotificationsParked() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notificationsParked');
  }

  static getNotificationsCharged() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notificationsCharged');
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

  static Future<bool> enableNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notifications', true);
  }

  static Future<bool> disableNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notifications', false);
  }

  static getNotifications() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getBool('notifications');
  }

  static setNotificationsParked(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notificationsParked', value);
  }

  static setNotificationsCharged(bool value) async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.setBool('notificationsCharged', value);
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
