import 'package:flutter/material.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';
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
    //TODO set vehicle provider to car provider
    for (int i = 0; i < _providers.length; i++) {}
  }

  //saves the selected tile
  void _setSelectedRadioTile(ChargingProvider value) {
    setState(() {
      _selectedRadioTile = value;
    });
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
}
