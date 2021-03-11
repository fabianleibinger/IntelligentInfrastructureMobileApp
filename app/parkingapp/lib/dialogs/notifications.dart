import 'package:flutter_local_notifications/flutter_local_notifications.dart';

//defines the push notifications sent by the app
class Notifications {
  static FlutterLocalNotificationsPlugin localNotification;

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

  static Future createNotification() async {
    _initialize();
    var androidDetails = new AndroidNotificationDetails(
        'channelId', 'local notification', 'basic notification',
        importance: Importance.high);
    var iOSDetails = new IOSNotificationDetails();
    var generalNotificationDetails =
        new NotificationDetails(android: androidDetails, iOS: iOSDetails);

    await localNotification.show(
        0, 'title', 'body', generalNotificationDetails);
  }
}
