import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/examplevehicle.dart';
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
  int _selectedRadioTile;
  List<ExampleVehicle> _exampleVehicles;

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
    return Column(children: <Widget>[
      FutureBuilder(
          future: DefaultAssetBundle.of(context)
              .loadString('assets/example-vehicles.json'),
          builder: (context, snapshot) {
            _exampleVehicles = _parseJson(snapshot.data.toString());
            _exampleVehicles.forEach((element) {
              if (element.height == vehicle.height &&
                  element.width == vehicle.width &&
                  element.length == vehicle.length) {
                _selectedRadioTile = _exampleVehicles.indexOf(element);
              }
            });
            return _exampleVehicles.isNotEmpty
                ? Container(
                    height: 150,
                    width: 250,
                    child: ListView.builder(
                      itemCount: _exampleVehicles == null
                          ? 0
                          : _exampleVehicles.length,
                      itemBuilder: (BuildContext context, int index) {
                        return RadioListTile(
                          value: index,
                          groupValue: _selectedRadioTile,
                          onChanged: (value) {
                            setState(() {
                              _setSelectedRadioTile(value);
                            });
                          },
                          title: Text(_exampleVehicles[index].name),
                        );
                      },
                    ))
                : Center(
                    child: CircularProgressIndicator(),
                  );
          })
    ]);
  }

  void _setSelectedRadioTile(int index) {
    _setExampleVehicle(vehicle, index);
    setState(() {
      _selectedRadioTile = index;
    });
  }

  void _setExampleVehicle(Vehicle vehicle, int index) {
    vehicle.setDimensions(context, _exampleVehicles[index].height,
        _exampleVehicles[index].width, _exampleVehicles[index].length);
  }

  List<ExampleVehicle> _parseJson(String response) {
    if (response == null) {
      return [];
    }
    List<ExampleVehicle> list;

    list = (json.decode(response) as List)
        .map((e) => ExampleVehicle.fromJson(e))
        .toList();

    return list;
  }
}
