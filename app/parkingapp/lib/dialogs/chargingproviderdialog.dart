import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

/// The Dialog that sets the charging provider for [ChargeableVehicles]
class ChargingProviderDialog extends StatefulWidget {
  @override
  _ChargingProviderDialogState createState() => _ChargingProviderDialogState();
}

class _ChargingProviderDialogState extends State<ChargingProviderDialog> {
  /// The list of all [ChargingProviders].
  List<ChargingProvider> _providers = ChargingProvider.values;

  /// The value of the selected [RadioListTile].
  ChargingProvider _selected;

  /// Sets the initially selected [RadioListTile].
  @override
  void initState() {
    super.initState();
    if (vehicle.runtimeType == ChargeableVehicle) {
      _selectChargingProvider(vehicle);
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

  /// Returns a [RadioListTile] for every charging provider.
  _getRadioListTiles(BuildContext context) {
    return Column(
      children: [
        for (int i = 0; i < _providers.length; i++)
          RadioListTile<ChargingProvider>(
              title: Text(_providers[i].toShortString()),
              value: _providers[i],
              groupValue: _selected,
              onChanged: (value) {
                _setSelectedRadioTile(value);
              })
      ],
    );
  }

  /// Selects the charging provider [value] and sets vehicles provider accordingly.
  void _setSelectedRadioTile(ChargingProvider value) {
    setState(() {
      _selected = value;
    });
    if (vehicle.runtimeType == ChargeableVehicle) {
      _setChargingProvider(vehicle);
    }
  }

  /// Selects the right charging Provider value for [_selectedRadioTile].
  void _selectChargingProvider(ChargeableVehicle vehicle) {
    _providers.forEach((provider) {
      if (vehicle.chargingProvider == provider.toShortString()) {
        _selected = provider;
      }
    });
  }

  /// Sets [vehicle] charging provider value.
  void _setChargingProvider(ChargeableVehicle vehicle) {
    vehicle.setAndUpdateChargingProvider(context, _selected.toShortString());
  }
}
