import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/examplevehicledimensions.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';

//defines the vehicle dimensions dialog
class VehicleDimensionsDialog extends StatefulWidget {
  VehicleDimensionsDialog({Key key}) : super(key: key);

  @override
  _VehicleDimensionsDialogState createState() =>
      _VehicleDimensionsDialogState();
}

class _VehicleDimensionsDialogState extends State<VehicleDimensionsDialog> {
  ExampleVehicleDimensions _selectedRadioTile;
  List<ExampleVehicleDimensions> _exampleVehicles =
      ExampleVehicleDimensions.instance;

  //sets the initially selected tile
  @override
  void initState() {
    super.initState();
    _checkVehicleDimensions(vehicle);
  }

  //saves the selected tile and changes vehicles values
  void _setSelectedRadioTile(ExampleVehicleDimensions exampleVehicle) {
    setState(() {
      _selectedRadioTile = exampleVehicle;
    });
    _setVehicleDimensions(vehicle, exampleVehicle);
  }

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
    return Column(children: [
      for (int i = 0; i < _exampleVehicles.length; i++)
        RadioListTile<ExampleVehicleDimensions>(
            title: Text(_exampleVehicles[i].name),
            value: _exampleVehicles[i],
            groupValue: _selectedRadioTile,
            onChanged: (value) {
              _setSelectedRadioTile(value);
            })
    ]);
  }

  //selects the right exampleVehicleDimensions value for _selectedRadioTile
  void _checkVehicleDimensions(Vehicle vehicle) {
    _exampleVehicles.forEach((exampleVehicle) {
      if (vehicle.height.compareTo(exampleVehicle.height) == 0 &&
          vehicle.width.compareTo(exampleVehicle.width) == 0 &&
          vehicle.length.compareTo(exampleVehicle.length) == 0 &&
          vehicle.turningCycle.compareTo(exampleVehicle.turningCycle) == 0) {
        _selectedRadioTile = exampleVehicle;
      }
    });
  }

  //sets vehicle dimension values
  void _setVehicleDimensions(
      Vehicle vehicle, ExampleVehicleDimensions exampleVehicle) {
    vehicle.setDimensions(context, exampleVehicle.height, exampleVehicle.width,
        exampleVehicle.length, exampleVehicle.turningCycle);
  }
}
