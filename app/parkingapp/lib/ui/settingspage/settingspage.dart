import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/notifications/notifications.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/settingspage/AGBpage.dart';
import 'package:parkingapp/ui/settingspage/passcodepage.dart';
import 'package:parkingapp/util/qrgenerator.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:system_settings/system_settings.dart';

/// Settings ui
///
/// User can modify push notification and passcode settings and can transfer
/// vehicles via QR from this page
class SettingsPage extends StatelessWidget {
  static const String routeName = '/settingspage';
  const SettingsPage({Key key}) : super(key: key);

  static final String notificationSettingKey = 'pushNotifications';
  static final String notificationLoadSettingKey = 'pushLoad';
  static final String notificationParkSettingKey = 'pushPark';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      appBar: AppBar(title: Text(AppLocalizations.of(context).settingsTitle)),
      body: SettingsForm(),
    );
  }
}

/// State for settings page
class SettingsForm extends StatefulWidget {
  @override
  State<StatefulWidget> createState() => SettingsFormState();
}

class SettingsFormState extends State<SettingsForm> {
  //Initialize all the settings
  bool _pushNotifications = false;
  bool _pushNotificationsParked = false;
  bool _pushNotificationsCharge = false;

  Future<SharedPreferences> preferences = SharedPreferences.getInstance();

  @override
  void initState() {
    super.initState();
    _getPushNotifications();
  }

  @override
  Widget build(BuildContext context) {
    return _settingsUi();
  }

  /// Creates ListView
  Widget _settingsUi() {
    return ListView(
      children: <Widget>[
        Padding(padding: EdgeInsets.fromLTRB(0, 10.0, 0, 0)),
        _notificationsListTile(),
        Divider(),
        _notificationsParkedListTile(),
        Divider(),
        _notificationsChargedListTile(),
        Divider(),
        _passcodeListTile(),
        Divider(),
        _transferDataListTile(),
        Divider(),
        _showTermsListTile()
      ],
    );
  }

  /// Creates ListTile for notifications and stores users preferences in sharedpreferences
  Widget _notificationsListTile() {
    return SwitchListTile(
        title: Text(AppLocalizations.of(context).pushMessages),
        subtitle: Text(AppLocalizations.of(context).pushMessagesText),
        value: _pushNotifications,
        onChanged: (value) async {
          if (value) {
            if (await Permission.notification.request().isDenied) {
              SystemSettings.app();
            }
            if (await Permission.notification.request().isDenied) {
              value = false;
            }
          }
          //update change in Sharedpreferences
          if (value) {
            SharedPreferencesHelper.enableNotifications();
          } else {
            SharedPreferencesHelper.disableNotifications();
          }
          setState(() {
            _pushNotifications = value;
          });
        });
  }

  /// Creates ListTile for notifications if vehicle is parked
  /// and stores users preferences in sharedpreferences
  Widget _notificationsParkedListTile() {
    return SwitchListTile(
        title: Text(AppLocalizations.of(context).pushMessagesParked),
        subtitle: Text(AppLocalizations.of(context).pushMessagesParkedText),
        value: _pushNotificationsParked && _pushNotifications,
        onChanged: (value) {
          SharedPreferencesHelper.setNotificationsParked(value);
          setState(() {
            _pushNotificationsParked = value;
          });
        });
  }

  /// Creates ListTile for notifications if vehicle is charged
  /// and stores users preferences in sharedpreferences
  Widget _notificationsChargedListTile() {
    return SwitchListTile(
        title: Text(AppLocalizations.of(context).pushMessagesCharge),
        subtitle: Text(AppLocalizations.of(context).pushMessagesChargeText),
        value: _pushNotificationsCharge && _pushNotifications,
        onChanged: (value) {
          SharedPreferencesHelper.setNotificationsCharged(value);
          setState(() {
            _pushNotificationsCharge = value;
          });
        });
  }

  /// Creates List Tile for passcode and calls _passCodeSettings()
  Widget _passcodeListTile() {
    return ListTile(
      title: Text(AppLocalizations.of(context).password),
      subtitle: Text(AppLocalizations.of(context).passwordSubtitle),
      trailing: Icon(Icons.arrow_forward_ios),
      onTap: () {
        _passCodeSettings();
      },
    );
  }

  /// Creates List Tile for transfer Data and routes to [TransferKeys]
  Widget _transferDataListTile() {
    return ListTile(
        title: Text(AppLocalizations.of(context).transferData),
        subtitle: Text(AppLocalizations.of(context).transferDataText),
        trailing: Icon(Icons.arrow_forward_ios),
        onTap: () {
          Navigator.pushNamed(context, Routes.transferKeys);
        });
  }

  /// Creates List Tile for showing terms and conditions and routes to [AGB]
  Widget _showTermsListTile() {
    return ListTile(
        title: Text(AppLocalizations.of(context).showTermsAndConditions),
        subtitle:
            Text(AppLocalizations.of(context).showTermsAndConditionsSubtitle),
        trailing: Icon(Icons.arrow_forward_ios),
        onTap: () {
          Navigator.pushNamed(context, Routes.agbPage);
        });
  }

  /// Routes to [PassCodePage]
  _passCodeSettings() {
    Navigator.push(context,
        MaterialPageRoute(builder: (BuildContext context) => PasscodePage()));
  }

  /// Gets all the shared preferences for initstate
  Future<Null> _getPushNotifications() async {
    final SharedPreferences prefs = await preferences;
    bool notifications = prefs.getBool('notifications');
    bool notificationsCharge = prefs.getBool('notificationsCharged');
    bool notificationsParked = prefs.getBool('notificationsParked');

    //check if values are already initialized
    if (notifications == null) {
      notifications = false;
    }
    if (notificationsCharge == null) {
      notificationsCharge = false;
    }
    if (notificationsParked == null) {
      notificationsParked = false;
    }

    Notifications.getEnabledValues();

    //sets state of settings page
    setState(() {
      _pushNotifications = notifications;
      _pushNotificationsCharge = notificationsCharge;
      _pushNotificationsParked = notificationsParked;
    });
  }
}
