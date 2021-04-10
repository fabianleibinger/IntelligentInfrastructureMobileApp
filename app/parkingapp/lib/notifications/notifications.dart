import 'package:flutter_local_notifications/flutter_local_notifications.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// The local notifications.
class Notifications {
  static FlutterLocalNotificationsPlugin _localNotification;

  /// The icon path.
  static String _notificationIcon = '@mipmap/ic_launcher';

  /// The Android-specific settings.
  static String _androidDetailsChannelID = 'channelId';
  static String _androidDetailsLocalNotification = 'local notification';
  static String _androidDetailsBasicNotification = 'basic notification';

  /// enable notifications.
  static bool _enabled;
  static bool _enabledForPark;
  static bool _enabledForCharge;

  /// Sets the initial notification settings.
  /// Needs to be called in every method that creates a new notification.
  static _initialize() async {
    var androidInitialize =
        new AndroidInitializationSettings(_notificationIcon);
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    _localNotification = new FlutterLocalNotificationsPlugin();
    await _localNotification.initialize(initializationSettings);
  }

  /// Sets the initial clickable notification settings.
  /// The Function to be called by notification [onSelectedNotification].
  /// Needs to be called in every method that creates a new clickable notification.
  static _initializeClickable(
      Future<dynamic> Function(String) onSelectedNotification) async {
    var androidInitialize =
        new AndroidInitializationSettings(_notificationIcon);
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    _localNotification = new FlutterLocalNotificationsPlugin();
    await _localNotification.initialize(initializationSettings,
        onSelectNotification: onSelectedNotification);
  }

  /// Creates a notification with [title], [body].
  /// enables notification according to operation type: [parkOperation], [chargeOperation].
  static Future createNotification(String title, String body,
      bool parkOperation, bool chargeOperation) async {
    if (_checkEnabled(parkOperation, chargeOperation)) {
      _initialize();
      var androidDetails = new AndroidNotificationDetails(
          _androidDetailsChannelID,
          _androidDetailsLocalNotification,
          _androidDetailsBasicNotification,
          importance: Importance.high);
      var iOSDetails = new IOSNotificationDetails();
      var generalNotificationDetails =
      new NotificationDetails(android: androidDetails, iOS: iOSDetails);

      await _localNotification.show(0, title, body, generalNotificationDetails);
    }
  }

  /// Creates a clickable notification
  /// with [title], [body], [payload], [onSelectedNotification],
  /// enables notification according to operation type: [parkOperation], [chargeOperation].
  static Future createNotificationClickable(
      String title,
      String body,
      String payload,
      Future<dynamic> Function(String) onSelectedNotification,
      bool parkOperation,
      bool chargeOperation) async {
    if (_checkEnabled(parkOperation, chargeOperation)) {
      _initializeClickable(onSelectedNotification);
      var androidDetails = new AndroidNotificationDetails(
          _androidDetailsChannelID,
          _androidDetailsLocalNotification,
          _androidDetailsBasicNotification,
          importance: Importance.high);
      var iOSDetails = new IOSNotificationDetails();
      var generalNotificationDetails =
      new NotificationDetails(android: androidDetails, iOS: iOSDetails);

      await _localNotification.show(0, title, body, generalNotificationDetails,
          payload: payload);
    }
  }

  /// Checks and returns if notification is enabled according to operation type:
  /// [parkOperation], [chargeOperation].
  static bool _checkEnabled(bool parkOperation, bool chargeOperation) {
    getEnabledValues();
    if (_enabled &&
        ((parkOperation && _enabledForPark) ||
            (chargeOperation && _enabledForCharge))) {
      return true;
    } else {
      return false;
    }
  }

  /// Gets values for the enabled attributes from [SettingsPage].
  static getEnabledValues() async {
    final prefs = await SharedPreferences.getInstance();
    _enabled = prefs.getBool('notifications') ?? false;
    _enabledForPark = prefs.getBool('notificationsParked') ?? false;
    _enabledForCharge = prefs.getBool('notificationsCharged') ?? false;
  }

}
