import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

/// The different QR Code-related dialogs.
///
/// ```showDialog(
///             context: context,
///             builder: (context) {
///               return ScanQRDialogs.getVehicleQRDialog(context);
///             });
/// ```
class ScanQRDialogs {

  //TODO: update comment
  /// Returns the dialog that opens [].
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

  /// Returns the dialog that tells the User that QR code creation is not possible
  /// because the selected vehicle is currently parking in or out.
  static getNoScanQRPossibleDialog(BuildContext context) {
    return Constants.getAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).noScanPossibleDialogText,
        AppLocalizations.of(context).parkDialogBackButton);
  }
}