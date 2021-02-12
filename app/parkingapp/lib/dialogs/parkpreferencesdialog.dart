import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
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
    // TODO: implement initState
    super.initState();
    _nearExitCheckBox = false;
    _parkingCardCheckBox = false;
  }

  void _setNearExitCheckboxValue(bool value) {
    setState(() {
      _nearExitCheckBox = value;
    });
  }

  void _setParkingCardCheckboxValue(bool value) {
    setState(() {
      _parkingCardCheckBox = value;
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
