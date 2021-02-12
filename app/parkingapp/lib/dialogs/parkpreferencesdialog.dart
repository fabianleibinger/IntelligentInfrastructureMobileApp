import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

class ParkPreferencesDialog extends StatefulWidget {
  ParkPreferencesDialog({Key key}) : super(key: key);

  @override
  _ParkPreferencesDialogState createState() => _ParkPreferencesDialogState();
}

class _ParkPreferencesDialogState extends State<ParkPreferencesDialog> {
  static bool _nearExitCheckBox;
  static bool _parkingCardCheckBox;

  //sets the initial check box values
  @override
  void initState() {
    // TODO: switch to correct currentVehicle
    super.initState();
    _nearExitCheckBox = currentVehicle.nearExitPreference;
    _parkingCardCheckBox = currentVehicle.parkingCard;
  }

  //switches current vehicles value and checkbox value
  void _setNearExitCheckboxValue(bool value) {
    setState(() {
      currentVehicle.nearExitPreference = value;
      _nearExitCheckBox = currentVehicle.nearExitPreference;
    });
  }

  //switches current vehicles value and checkbox value
  void _setParkingCardCheckboxValue(bool value) {
    setState(() {
      currentVehicle.parkingCard = value;
      _parkingCardCheckBox = currentVehicle.parkingCard;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        'Parkpräferenzen',
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
            Text('Nahe am Ausgang')
          ],
        ),
        Row(
          children: [
            Checkbox(
                value: _parkingCardCheckBox,
                onChanged: (value) {
                  _setParkingCardCheckboxValue(value);
                }),
            Text('Dauerparkkarte')
          ],
        )
      ],
    );
  }
}
