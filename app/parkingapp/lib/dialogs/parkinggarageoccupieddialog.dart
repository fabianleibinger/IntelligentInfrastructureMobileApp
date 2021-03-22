import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

/// The dialog that tells the user that the parking garage is occupied.
///
/// ```showDialog(
///             context: context,
///             builder: (context) {
///               return ParkingGarageOccupiedDialog.getDialog(context);
///             });
/// ```
class ParkingGarageOccupiedDialog {

  /// Returns dialog that opens the correct page for the vehicle.
  static getDialog(BuildContext context) {
    return Constants.getAlertDialogOneButtonNoTitle(
        context,
        AppLocalizations.of(context).parkingGarageOccupiedDialogContent,
        AppLocalizations.of(context).parkDialogBackButton,
        vehicle.inAppKey);
  }
}
