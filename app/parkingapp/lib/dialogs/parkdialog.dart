import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';

//defines the different park-related dialogs
class ParkDialog {
  static createParkInDialog(BuildContext context) {
    return Constants.createAlertDialog(
        context,
        AppLocalizations.of(context).parkDialogParkInTitle,
        AppLocalizations.of(context).parkDialogParkInContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkInButton,
        vehicle.inAppKey + Routes.parkIn);
  }

  static createParkOutDialog(BuildContext context) {
    return Constants.createAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        vehicle.inAppKey + Routes.parkOut);
  }

  static createParkInCancelDialog(BuildContext context) {
    return Constants.createAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkCancelContent,
        AppLocalizations.of(context).parkDialogBackButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        vehicle.inAppKey + Routes.parkOut);
  }

  static createParkOutFinishedDialog(BuildContext context) {
    return Constants.createAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutFinishedContent,
        AppLocalizations.of(context).dialogFinishedButton);
  }
}
