import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/FirstStart/addvehicle.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/settingspage/passcodepage.dart';
import 'package:parkingapp/util/qrscanner.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:system_settings/system_settings.dart';

/// Class for settingspage in first start procedure
class AppConfiguration extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: AppDrawer(),
      appBar: AppBar(
          title: Text(AppLocalizations.of(context).titleConfigureApp)),
      bottomNavigationBar: Material(
        elevation: 10,
        child: ButtonBar(
          children: [
            TextButton(
              child: Text(AppLocalizations.of(context).buttonContinue),
              onPressed: () => Navigator.push(context,
                  MaterialPageRoute(builder: (context) => AddVehicle())),
            )
          ],
        ),
      ),
      body: AppConfigurationForm(),
    );
  }
}

class AppConfigurationForm extends StatefulWidget {
  @override
  _AppConfigurationState createState() => _AppConfigurationState();
}

/// State for first start app configurations
class _AppConfigurationState extends State<AppConfigurationForm> {
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
        _importVehicleListTile(),
        Divider(),
        _showTermsListTile()
      ],
    );
  }

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

  Widget _importVehicleListTile() {
    return ListTile(
        title: Text(AppLocalizations.of(context).importVehicles),
        subtitle: Text(AppLocalizations.of(context).importVehiclesSubtitle),
        trailing: Icon(Icons.arrow_forward_ios),
        onTap: () {
          Navigator.push(
              context,
              MaterialPageRoute(
                  builder: (BuildContext context) => ScanScreen()));
        });
  }

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

  _passCodeSettings() {
    Navigator.push(context,
        MaterialPageRoute(builder: (BuildContext context) => PasscodePage()));
  }

  /// Gets all the shared prefernces for initstate
  Future<Null> _getPushNotifications() async {
    //inialize SharedPreferences Settings with false values
    SharedPreferencesHelper.initializeSharedPreferences();

    setState(() {
      _pushNotifications = false;
      _pushNotificationsCharge = false;
      _pushNotificationsParked = false;
    });
  }
}
