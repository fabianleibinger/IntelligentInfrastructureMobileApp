import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//defines charging provider dialog
class ChargingProviderDialog extends StatefulWidget {
  @override
  _ChargingProviderDialogState createState() => _ChargingProviderDialogState();
}

class _ChargingProviderDialogState extends State<ChargingProviderDialog> {
  List<ChargingProvider> _providers = ChargingProvider.values;
  static ChargingProvider _selectedRadioTile;

  //sets the initially selected tile
  @override
  void initState() {
    super.initState();
    if (vehicle.runtimeType == LoadableVehicle) {
      _checkChargingProvider(vehicle);
    }
  }

  //saves the selected tile and changes vehicles value
  void _setSelectedRadioTile(ChargingProvider value) {
    setState(() {
      _selectedRadioTile = value;
    });
    if (vehicle.runtimeType == LoadableVehicle) {
      _setChargingProvider(vehicle);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations.of(context).chargingProviderDialogTitle,
        AppLocalizations.of(context).settingsSaveButton,
        _getRadioListTiles(context));
  }

  //returns a tile for every charging provider
  _getRadioListTiles(BuildContext context) {
    return Column(
      children: [
        for (int i = 0; i < _providers.length; i++)
          RadioListTile<ChargingProvider>(
              title: Text(_providers[i].toShortString()),
              value: _providers[i],
              groupValue: _selectedRadioTile,
              onChanged: (value) {
                _setSelectedRadioTile(value);
              })
      ],
    );
  }

  //selects the right charging Provider value for _selectedRadioTile
  void _checkChargingProvider(LoadableVehicle vehicle) {
    for (int i = 0; i < _providers.length; i++) {
      if (vehicle.chargingProvider == _providers[i].toShortString()) {
        _selectedRadioTile = _providers[i];
      }
    }
  }

  //sets vehicle charging provider value
  void _setChargingProvider(LoadableVehicle vehicle) {
    vehicle.chargingProvider = _selectedRadioTile.toShortString();
    DatabaseProvider.db.update(vehicle);
  }
}
