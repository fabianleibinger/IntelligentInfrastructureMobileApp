import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class ChargingProviderDialog {
  static createChargingProviderDialog(BuildContext context) {
    return Constants.createConfirmationDialog(
        context,
        AppLocalizations.of(context).chargingProviderDialogTitle,
        AppLocalizations.of(context).settingsSaveButton,
        getRadioListTiles(context));
  }

  static getRadioListTiles(BuildContext context) {
    return Container(
      child: Column(
        children: [
          RadioListTile(
              title: Text(AppLocalizations.of(context)
                  .chargingProviderDialogTextAutomatic),
              value: 1,
              groupValue: 1,
              onChanged: (_) {}),
          for (int i = 0;
              i < currentParkingGarage.chargingProviders.length;
              i++)
            RadioListTile(
                title: Text(currentParkingGarage.chargingProviders[i]),
                value: 0,
                groupValue: 1,
                onChanged: (_) {})
        ],
      ),
    );
  }
}
