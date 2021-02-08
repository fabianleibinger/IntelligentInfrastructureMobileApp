import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class ChargingProviderDialog extends StatefulWidget {
  @override
  _ChargingProviderDialogState createState() => _ChargingProviderDialogState();
}

class _ChargingProviderDialogState extends State<ChargingProviderDialog> {
  List<ChargingProvider> _providers = ChargingProvider.values;
  static ChargingProvider _selectedRadioTile;

  @override
  initState() {
    super.initState();
    //TODO set vehicle provider to car provider
    for (int i = 0; i < _providers.length; i++) {

    }
  }

  setSelectedRadioTile(ChargingProvider value) {
    setState(() {
      _selectedRadioTile = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations
            .of(context)
            .chargingProviderDialogTitle,
        AppLocalizations
            .of(context)
            .settingsSaveButton,
        _getRadioListTiles(context));
  }

  _getRadioListTiles(BuildContext context) {
    return Container(
      child: Column(
        children: [
          for (int i = 0; i < _providers.length; i++)
            RadioListTile<ChargingProvider>(
                title: Text(_providers[i].toShortString()),
                value: _providers[i],
                groupValue: _selectedRadioTile,
                onChanged: (value) {
                  setSelectedRadioTile(value);
                })
        ],
      ),
    );
  }
}
