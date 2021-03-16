import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';

//defines the different park-related dialogs
class ParkDialog {

  //returns dialog, use prior to park in page
  static getParkInDialog(BuildContext context) {
    return Constants.getAlertDialog(
        context,
        AppLocalizations.of(context).parkDialogParkInTitle,
        AppLocalizations.of(context).parkDialogParkInContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkInButton,
        vehicle.inAppKey + Routes.parkIn);
  }

  //returns dialog, use prior to park out page
  static getParkOutDialog(BuildContext context) {
    return Constants.getAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        vehicle.inAppKey + Routes.parkOut);
  }

  //returns dialog, use prior to park out page cancel park in
  static getParkInCancelDialog(BuildContext context) {
    return Constants.getAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkCancelContent,
        AppLocalizations.of(context).parkDialogBackButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        vehicle.inAppKey + Routes.parkOut);
  }

  //returns dialog, use after park out finished
  static getParkOutFinishedDialog(BuildContext context) {
    return Constants.getAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutFinishedContent,
        AppLocalizations.of(context).dialogFinishedButton);
  }
}
