import 'package:flutter_local_notifications/flutter_local_notifications.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:shared_preferences_settings/shared_preferences_settings.dart';

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
  static bool _enabledForPark;
  static bool _enabledForCharge;

  /// Sets the initial notification settings.
  /// Needs to be called in every method that creates a new notification.
  static _initialize() {
    var androidInitialize =
        new AndroidInitializationSettings(_notificationIcon);
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    _localNotification = new FlutterLocalNotificationsPlugin();
    _localNotification.initialize(initializationSettings);
  }

  /// Sets the initial clickable notification settings.
  /// The Function to be called by notification [onSelectedNotification].
  /// Needs to be called in every method that creates a new clickable notification.
  static _initializeClickable(
      Future<dynamic> Function(String) onSelectedNotification) {
    var androidInitialize =
        new AndroidInitializationSettings(_notificationIcon);
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    _localNotification = new FlutterLocalNotificationsPlugin();
    _localNotification.initialize(initializationSettings,
        onSelectNotification: onSelectedNotification);
  }

  /// Creates a notification with [title], [body].
  static Future createNotification(String title, String body) async {
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

  /// Creates a clickable notification
  /// with [title], [body], [payload], [onSelectedNotification].
  static Future createNotificationClickable(
      String title,
      String body,
      String payload,
      Future<dynamic> Function(String) onSelectedNotification) async {
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

  /// Checks if notifications are enabled for charge information.
  static bool _checkEnabledForCharge() {
    _getEnabledValues();
    return _enabledForCharge;
  }

  /// Checks if notifications are enabled for park operations.
  static bool _checkEnabledForPark() {
    _getEnabledValues();
    return _enabledForPark;
  }

  /// Gets values for the enabled attributes from [SettingsPage].
  static _getEnabledValues() {
    _enabledForPark =
        Settings().getBool(SettingsPage.notificationParkSettingKey, false) ??
            false;
    _enabledForCharge =
        Settings().getBool(SettingsPage.notificationLoadSettingKey, false) ??
            false;
  }
}
