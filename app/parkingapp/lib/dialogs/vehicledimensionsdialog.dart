import 'package:flutter/material.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//defines the vehicle dimensions dialog
class VehicleDimensionsDialog extends StatefulWidget {
  VehicleDimensionsDialog({Key key}) : super(key: key);

  @override
  _VehicleDimensionsDialogState createState() =>
      _VehicleDimensionsDialogState();
}

class _VehicleDimensionsDialogState extends State<VehicleDimensionsDialog> {
  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations.of(context).vehicleDimensionsDialogTitle,
        AppLocalizations.of(context).dialogFinishedButton,
        _getRadioListTiles(context));
  }

  //returns a tile for every different dimension
  _getRadioListTiles(BuildContext context) {
    return Column(
      children: [],
    );
  }
}
