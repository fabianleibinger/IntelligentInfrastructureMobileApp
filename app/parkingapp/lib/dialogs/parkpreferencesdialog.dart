import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

//defines the park preferences dialog
class ParkPreferencesDialog extends StatefulWidget {
  ParkPreferencesDialog({Key key}) : super(key: key);

  @override
  _ParkPreferencesDialogState createState() => _ParkPreferencesDialogState();
}

class _ParkPreferencesDialogState extends State<ParkPreferencesDialog> {
  static bool _nearExitCheckBox;
  static bool _parkingCardCheckBox;

  //sets the initial check box values using the current vehicles attributes
  @override
  void initState() {
    // TODO: switch to correct currentVehicle
    super.initState();
    _nearExitCheckBox = vehicle.nearExitPreference;
    _parkingCardCheckBox = vehicle.parkingCard;
  }

  //switches current vehicles value and checkbox value
  void _setNearExitCheckboxValue(bool value) {
    setState(() {
      vehicle.nearExitPreference = value;
      DatabaseProvider.db.update(vehicle);
      _nearExitCheckBox = vehicle.nearExitPreference;
    });
  }

  //switches current vehicles value and checkbox value
  void _setParkingCardCheckboxValue(bool value) {
    setState(() {
      vehicle.parkingCard = value;
      DatabaseProvider.db.update(vehicle);
      _parkingCardCheckBox = vehicle.parkingCard;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations.of(context).parkPreferencesDialogTitle,
        AppLocalizations.of(context).settingsSaveButton,
        _getCheckBoxes(context));
  }

  //returns two checkboxes for near exit preference and parking card
  _getCheckBoxes(BuildContext context) {
    return Column(
      children: [
        Row(
          children: [
            Checkbox(
                value: _nearExitCheckBox,
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
                value: _parkingCardCheckBox,
                onChanged: (value) {
                  _setParkingCardCheckboxValue(value);
                }),
            Text(AppLocalizations.of(context).parkPreferencesDialogParkingCard)
          ],
        )
      ],
    );
  }
}
