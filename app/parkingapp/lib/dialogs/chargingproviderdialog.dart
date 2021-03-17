import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
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
  ChargingProvider _selectedRadioTile;

  //sets the initially selected tile
  @override
  void initState() {
    super.initState();
    if (vehicle.runtimeType == ChargeableVehicle) {
      _checkChargingProvider(vehicle);
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

  //saves the selected tile and changes vehicles value
  void _setSelectedRadioTile(ChargingProvider value) {
    setState(() {
      _selectedRadioTile = value;
    });
    if (vehicle.runtimeType == ChargeableVehicle) {
      _setChargingProvider(vehicle);
    }
  }

  //selects the right charging Provider value for _selectedRadioTile
  void _checkChargingProvider(ChargeableVehicle vehicle) {
    _providers.forEach((provider) {
      if(vehicle.chargingProvider == provider.toShortString()) {
        _selectedRadioTile = provider;
      }
    });
  }

  //sets vehicle charging provider value
  void _setChargingProvider(ChargeableVehicle vehicle) {
    vehicle.setChargingProvider(context, _selectedRadioTile.toShortString());
  }
}
