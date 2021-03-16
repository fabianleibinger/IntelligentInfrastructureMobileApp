import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/examplevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
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
  ExampleVehicle _selectedRadioTile;
  List<ExampleVehicle> _exampleVehicles;

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations.of(context).vehicleDimensionsDialogTitle,
        AppLocalizations.of(context).dialogFinishedButton,
        Column(children: [_getRadioListTiles(context), _customDimensions()]));
  }

  //returns a tile for every different dimension
  _getRadioListTiles(BuildContext context) {
    return Column(children: <Widget>[
      FutureBuilder(
          future: DefaultAssetBundle.of(context)
              .loadString('assets/example-vehicles.json'),
          builder: (context, snapshot) {
            if (snapshot.connectionState == ConnectionState.done) {
              _exampleVehicles = _parseJson(snapshot.data.toString());
              _exampleVehicles.forEach((element) {
                if (element.height == vehicle.height &&
                    element.width == vehicle.width &&
                    element.length == vehicle.length &&
                    element.turningCycle == vehicle.turningCycle &&
                    element.distRearAxleLicensePlate ==
                        vehicle.distRearAxleLicensePlate) {
                  _selectedRadioTile = element;
                }
              });
              return Container(
                  height: 150,
                  width: 250,
                  child: ListView.builder(
                    itemCount:
                        _exampleVehicles == null ? 0 : _exampleVehicles.length,
                    itemBuilder: (BuildContext context, int index) {
                      return RadioListTile<ExampleVehicle>(
                        value: _exampleVehicles[index],
                        groupValue: _selectedRadioTile,
                        onChanged: (value) {
                          setState(() {
                            _setSelectedRadioTile(value);
                          });
                        },
                        title: Text(_exampleVehicles[index].name),
                      );
                    },
                  ));
            } else {
              return CircularProgressIndicator();
            }
          })
    ]);
  }

  void _setSelectedRadioTile(ExampleVehicle exampleVehicle) {
    _setExampleVehicle(vehicle, exampleVehicle);
    setState(() {
      _selectedRadioTile = exampleVehicle;
    });
  }

  void _setExampleVehicle(Vehicle vehicle, ExampleVehicle exampleVehicle) {
    //update vehicle dimensions of the vehicle in the database with the new dimensions of exampleVehicle
    vehicle.setDimensions(
        context,
        exampleVehicle.width,
        exampleVehicle.height,
        exampleVehicle.length,
        exampleVehicle.turningCycle,
        exampleVehicle.distRearAxleLicensePlate);
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

  List<ExampleVehicle> parseJson(String response) {
    if (response == null) {
      return [];
    }
    List<ExampleVehicle> list;
    /*final parsed = json.decode(response);
    list = List<ExampleVehicle>.from(
        parsed.map((model) => ExampleVehicle.fromJson(model)));*/

    list = (json.decode(response) as List)
        .map((e) => ExampleVehicle.fromJson(e))
        .toList();

    return list;
  }

  //Dialog that allows entering of custom dimensions
  ListTile _customDimensions() {
    final _formKey = GlobalKey<FormState>();
    return ListTile(
      title: Text(AppLocalizations.of(context).customDimensions),
      onTap: () async {
        await showDialog(
            context: context,
            builder: (context) => AlertDialog(
                  title: Text(AppLocalizations.of(context).customDimensions),
                  content: Form(
                    key: _formKey,
                    child: ListView(
                      children: [
                        //length
                        TextFormField(
                          initialValue: vehicle.length.round().toString(),
                          keyboardType: TextInputType.number,
                          decoration: InputDecoration(
                              labelText: AppLocalizations.of(context).length +
                                  AppLocalizations.of(context).space +
                                  AppLocalizations.of(context)
                                      .additionalInMilimeters),
                          validator: (val) => (num.tryParse(val) ?? 0) > 0
                              ? null
                              : AppLocalizations.of(context).requiredText,
                          onSaved: (val) =>
                              vehicle.setLength(context, double.tryParse(val)),
                        ),
                        //width
                        TextFormField(
                          initialValue: vehicle.width.round().toString(),
                          keyboardType: TextInputType.number,
                          decoration: InputDecoration(
                              labelText: AppLocalizations.of(context).width +
                                  AppLocalizations.of(context).space +
                                  AppLocalizations.of(context)
                                      .additionalInMilimeters),
                          validator: (val) => (num.tryParse(val) ?? 0) > 0
                              ? null
                              : AppLocalizations.of(context).requiredText,
                          onSaved: (val) =>
                              vehicle.setWidth(context, double.tryParse(val)),
                        ),
                        //height
                        TextFormField(
                          initialValue: vehicle.height.round().toString(),
                          keyboardType: TextInputType.number,
                          decoration: InputDecoration(
                              labelText: AppLocalizations.of(context).height +
                                  AppLocalizations.of(context).space +
                                  AppLocalizations.of(context)
                                      .additionalInMilimeters),
                          validator: (val) => (num.tryParse(val) ?? 0) > 0
                              ? null
                              : AppLocalizations.of(context).requiredText,
                          onSaved: (val) =>
                              vehicle.setHeight(context, double.tryParse(val)),
                        ),
                        //turning circle
                        TextFormField(
                          initialValue: vehicle.turningCycle.round().toString(),
                          keyboardType: TextInputType.number,
                          decoration: InputDecoration(
                              labelText:
                                  AppLocalizations.of(context).turningCircle +
                                      AppLocalizations.of(context).space +
                                      AppLocalizations.of(context)
                                          .additionalInMilimeters),
                          validator: (val) => (num.tryParse(val) ?? 0) > 0
                              ? null
                              : AppLocalizations.of(context).requiredText,
                          onSaved: (val) => vehicle.setTurningCycle(
                              context, double.tryParse(val)),
                        ),
                        //distance rear axle license plate
                        TextFormField(
                          initialValue: vehicle.distRearAxleLicensePlate
                              .round()
                              .toString(),
                          keyboardType: TextInputType.number,
                          decoration: InputDecoration(
                              labelText: AppLocalizations.of(context)
                                      .distRearAxleLicensePlate +
                                  AppLocalizations.of(context).space +
                                  AppLocalizations.of(context)
                                      .additionalInMilimeters),
                          validator: (val) => (num.tryParse(val) ?? 0) > 0
                              ? null
                              : AppLocalizations.of(context).requiredText,
                          onSaved: (val) => vehicle.setDistRearAxleLicensePlate(
                              context, double.tryParse(val)),
                        )
                      ],
                    ),
                  ),
                  actions: [
                    FlatButton(
                      child: Text(AppLocalizations.of(context).buttonOk),
                      onPressed: () {
                        //validate form and exit on success
                        if (_formKey.currentState.validate()) {
                          _formKey.currentState.save();

                          Navigator.of(context).pop();
                        }
                      },
                    )
                  ],
                ));
        Navigator.of(context).pop();
      },
    );
  }
}
