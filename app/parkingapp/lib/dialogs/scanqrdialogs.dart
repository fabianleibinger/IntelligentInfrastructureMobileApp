import 'package:flutter/material.dart';
import 'package:parkingapp/routes/routes.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

/// The QR code dialogs
///
/// ```showDialog(
///             context: context,
///             builder: (context) {
///               return ScanQRDialogs.getVehicleQRDialog(context);
///             });
/// ```
class ScanQRDialogs {

  //TODO: comment
  /// Returns dialog that opens [].
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

  /// Returns the dialog that tells the User that a QR Code can't be created
  /// because the vehicle ist currently parking in or out.
  static getNoScanQRPossibleDialog(BuildContext context) {
    return Constants.getAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).noScanPossibleDialogText,
        AppLocalizations.of(context).parkDialogBackButton);
  }
}