import 'package:flutter_local_notifications/flutter_local_notifications.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';

//local notifications
class Notifications {
  static FlutterLocalNotificationsPlugin _localNotification;

  static bool _enabled;
  static bool _enabledForPark;
  static bool _enabledForCharge;

  //needs to be called in every method that creates a new notification.
  //sets the initial notification settings
  static _initialize() {
    var androidInitialize =
        new AndroidInitializationSettings('@mipmap/ic_launcher');
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    _localNotification = new FlutterLocalNotificationsPlugin();
    _localNotification.initialize(initializationSettings);
  }

  //needs to be called in every method that creates a new notification.
  //sets the initial notification settings
  //[onSelectedNotification]
  static _initializeClickable(
      Future<dynamic> Function(String) onSelectedNotification) {
    var androidInitialize =
        new AndroidInitializationSettings('@mipmap/ic_launcher');
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    _localNotification = new FlutterLocalNotificationsPlugin();
    _localNotification.initialize(initializationSettings,
        onSelectNotification: onSelectedNotification);
  }

  //creates notification with [title], [body]
  static Future createNotification(String title, String body) async {
    if (_checkEnabled()) {
      _initialize();
      var androidDetails = new AndroidNotificationDetails(
          'channelId', 'local notification', 'basic notification',
          importance: Importance.high);
      var iOSDetails = new IOSNotificationDetails();
      var generalNotificationDetails =
          new NotificationDetails(android: androidDetails, iOS: iOSDetails);

      await _localNotification.show(0, title, body, generalNotificationDetails);
    }
  }

  //creates notification with [title], [body], [payload], [onSelectedNotification]
  static Future createNotificationClickable(
      String title,
      String body,
      String payload,
      Future<dynamic> Function(String) onSelectedNotification) async {
    if (_checkEnabled()) {
      _initializeClickable(onSelectedNotification);
      var androidDetails = new AndroidNotificationDetails(
          'channelId', 'local notification', 'basic notification',
          importance: Importance.high);
      var iOSDetails = new IOSNotificationDetails();
      var generalNotificationDetails =
          new NotificationDetails(android: androidDetails, iOS: iOSDetails);

      await _localNotification.show(0, title, body, generalNotificationDetails,
          payload: payload);
    }
  }

  static bool _checkEnabled() {
    _getEnabledValues();
    return _enabled;
  }

  static bool _checkEnabledForCharge() {
    _getEnabledValues();
    return _enabledForCharge;
  }

  static bool _checkEnabledForPark() {
    _getEnabledValues();
    return _enabledForPark;
  }

  //gets values for the enabled attributes from Settings
  static _getEnabledValues() {
    _enabled =
        Settings().getBool(SettingsPage.notificationSettingKey, false) ?? false;
    _enabledForPark =
        Settings().getBool(SettingsPage.notificationParkSettingKey, false) ??
            false;
    _enabledForCharge =
        Settings().getBool(SettingsPage.notificationLoadSettingKey, false) ??
            false;
  }
}
