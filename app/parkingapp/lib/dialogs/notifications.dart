import 'package:flutter_local_notifications/flutter_local_notifications.dart';

//local notifications
class Notifications {
  static FlutterLocalNotificationsPlugin localNotification;

  static bool enabled;
  static bool enabledForPark;
  static bool enabledForCharge;

  //needs to be called in every method that creates a new notification.
  //sets the initial notification settings
  static _initialize() {
    var androidInitialize =
    new AndroidInitializationSettings('@mipmap/ic_launcher');
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    localNotification = new FlutterLocalNotificationsPlugin();
    localNotification.initialize(initializationSettings);
  }

  //needs to be called in every method that creates a new notification.
  //sets the initial notification settings
  //[onSelectedNotification]
  static _initializeClickable(Future<dynamic> Function(String) onSelectedNotification) {
    var androidInitialize =
        new AndroidInitializationSettings('@mipmap/ic_launcher');
    var iOSInitialize = new IOSInitializationSettings();
    var initializationSettings = new InitializationSettings(
        android: androidInitialize, iOS: iOSInitialize);

    localNotification = new FlutterLocalNotificationsPlugin();
    localNotification.initialize(initializationSettings,
        onSelectNotification: onSelectedNotification);
  }

  //creates notification with [title], [body]
  static Future createNotification(String title, String body) async {
    _initialize();
    var androidDetails = new AndroidNotificationDetails(
        'channelId', 'local notification', 'basic notification',
        importance: Importance.high);
    var iOSDetails = new IOSNotificationDetails();
    var generalNotificationDetails =
        new NotificationDetails(android: androidDetails, iOS: iOSDetails);

    await localNotification.show(0, title, body, generalNotificationDetails);
  }

  //creates notification with [title], [body], [payload], [onSelectedNotification]
  static Future createNotificationClickable(String title, String body, String payload,
      Future<dynamic> Function(String) onSelectedNotification) async {
    _initializeClickable(onSelectedNotification);
    var androidDetails = new AndroidNotificationDetails(
        'channelId', 'local notification', 'basic notification',
        importance: Importance.high);
    var iOSDetails = new IOSNotificationDetails();
    var generalNotificationDetails =
    new NotificationDetails(android: androidDetails, iOS: iOSDetails);

    await localNotification.show(0, title, body, generalNotificationDetails,
        payload: payload);
  }
}
