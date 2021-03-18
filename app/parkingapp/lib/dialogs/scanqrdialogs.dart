import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//defines the qr code dialogs
class ScanQRDialogs {

  //returns dialog, use prior to qr code creation
  static getVehicleQRDialog(BuildContext context) {
    //TODO update next page
    return Constants.getAlertDialog(
        context,
        AppLocalizations.of(context).scanQRDialogTitle,
        AppLocalizations.of(context).scanQRDialogContent,
        AppLocalizations.of(context).scanQRDialogCancelButton,
        AppLocalizations.of(context).scanQRDialogConfirmButton,
        Routes.settings);
  }

  //returns the dialog for vehicles currently parking in or out
  static getNoScanQRPossibleDialog(BuildContext context) {
    return Constants.getAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).noScanPossibleDialogText,
        AppLocalizations.of(context).parkDialogBackButton);
  }
}