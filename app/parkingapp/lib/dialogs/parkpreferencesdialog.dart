import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

/// The Dialog that sets the park preferences for [Vehicles].
class ParkPreferencesDialog extends StatefulWidget {
  ParkPreferencesDialog({Key key}) : super(key: key);

  @override
  _ParkPreferencesDialogState createState() => _ParkPreferencesDialogState();
}

class _ParkPreferencesDialogState extends State<ParkPreferencesDialog> {

  /// The values of the [Checkboxes].
  bool _nearExit;
  bool _parkingCard;

  /// Sets the initial [Checkbox] values.
  @override
  void initState() {
    super.initState();
    _nearExit = vehicle.nearExitPreference;
    _parkingCard = vehicle.parkingCard;
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations.of(context).parkPreferencesDialogTitle,
        AppLocalizations.of(context).settingsSaveButton,
        _getCheckBoxes(context));
  }

  /// Returns two [Checkboxes] for near exit preference and parking card.
  _getCheckBoxes(BuildContext context) {
    return Column(
      children: [
        Row(
          children: [
            Checkbox(
                value: _nearExit,
                onChanged: (value) {
                  _setNearExitCheckboxValue(value);
                }),
            Text(AppLocalizations.of(context)
                .parkPreferencesDialogNearExitPreference)
          ],
        ),
        Row(
          children: [
            Checkbox(
                value: _parkingCard,
                onChanged: (value) {
                  _setParkingCardCheckboxValue(value);
                }),
            Text(AppLocalizations.of(context).parkPreferencesDialogParkingCard)
          ],
        )
      ],
    );
  }

  /// Switches [CheckBox] value and sets [vehicle.nearExitPreference].
  void _setNearExitCheckboxValue(bool value) {
    setState(() {
      vehicle.nearExitPreference = value;
      _nearExit = vehicle.nearExitPreference;
    });
    vehicle.setNearExitPreference(context, value);
  }

  /// Switches [CheckBox] value and sets [vehicle.parkingCard].
  void _setParkingCardCheckboxValue(bool value) {
    setState(() {
      vehicle.parkingCard = value;
      _parkingCard = vehicle.parkingCard;
    });
    vehicle.setParkingCard(context, value);
  }
}
