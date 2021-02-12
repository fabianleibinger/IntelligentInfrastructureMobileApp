import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//defines the qr code dialog
class VehicleQRDialog {
  static createVehicleQRDialog(BuildContext context) {
    //TODO update next page
    return Constants.createAlertDialog(
        context,
        AppLocalizations.of(context).vehicleQRDialogTitle,
        AppLocalizations.of(context).vehicleQRDialogContent,
        AppLocalizations.of(context).vehicleQRDialogCancelButton,
        AppLocalizations.of(context).vehicleQRDialogConfirmButton,
        Routes.settings);
  }
}
