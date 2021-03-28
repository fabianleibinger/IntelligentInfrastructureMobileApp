import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/settingspage/qrpage.dart';
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

  /// Returns the dialog that opens the [QRPage] for a specific [vehicle].
  static getVehicleQRDialog(BuildContext context, Vehicle vehicle) {
    return AlertDialog(
      title: Text(AppLocalizations.of(context).scanQRDialogTitle),
      content: Text(AppLocalizations.of(context).scanQRDialogContent),
      actions: [
        Constants.getBackTextButton(context, red,
            AppLocalizations.of(context).scanQRDialogCancelButton),
        FlatButton(
          textColor: green,
          onPressed: () => Navigator.pushReplacement(
              context,
              MaterialPageRoute(
                  builder: (BuildContext context) => QRPage(vehicle: vehicle))),
          child: Text(AppLocalizations.of(context).scanQRDialogConfirmButton),
        )
      ],
    );
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
