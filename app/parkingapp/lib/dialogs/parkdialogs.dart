import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/parkpages/parkinpage.dart';
import 'package:parkingapp/ui/parkpages/parkoutpage.dart';

/// The different park-related dialogs.
///
/// ```showDialog(
///             context: context,
///             builder: (context) {
///               return ParkDialogs.getParkInDialog(context);
///             });
/// ```
class ParkDialogs {

  /// Returns park in dialog that opens [ParkInPage].
  static getParkInDialog(BuildContext context) {
    return Constants.getAlertDialog(
        context,
        AppLocalizations.of(context).parkDialogParkInTitle,
        AppLocalizations.of(context).parkDialogParkInContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkInButton,
        vehicle.inAppKey + Routes.parkIn);
  }

  /// Returns park out dialog that opens [ParkOutPage].
  static getParkOutDialog(BuildContext context) {
    return Constants.getAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        vehicle.inAppKey + Routes.parkOut);
  }

  /// Returns cancel park in dialog that opens [ParkOutPage].
  static getParkInCancelDialog(BuildContext context) {
    return Constants.getAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkCancelContent,
        AppLocalizations.of(context).parkDialogBackButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        vehicle.inAppKey + Routes.parkOut);
  }

  static getParkOutFinishedDialog(BuildContext context) {
    return Constants.getAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutFinishedContent,
        AppLocalizations.of(context).dialogFinishedButton);
  }
}
