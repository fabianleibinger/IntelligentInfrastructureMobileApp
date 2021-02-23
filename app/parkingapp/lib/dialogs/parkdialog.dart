import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/global.dart';
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
        Routes.park);
  }

  static createParkOutDialog(BuildContext context) {
    return Constants.createAlertDialogNoTitleAssignButton(
        context,
        AppLocalizations.of(context).parkDialogParkOutContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        getParkOutButton(context));
  }

  static createParkInCancelDialog(BuildContext context) {
    return Constants.createAlertDialogNoTitleAssignButton(
        context,
        AppLocalizations.of(context).parkDialogParkCancelContent,
        AppLocalizations.of(context).parkDialogBackButton,
        getParkOutButton(context));
  }

  static createParkOutFinishedDialog(BuildContext context) {
    return Constants.createAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutFinishedContent,
        AppLocalizations.of(context).dialogFinishedButton);
  }
  
  //button that triggers vehicle park out method
  static getParkOutButton(BuildContext context) {
    return FlatButton(
      textColor: green,
      onPressed: () {
        vehicle.parkOut(context);
        Navigator.pushReplacementNamed(context, Routes.park);
      },
      child: Text(AppLocalizations.of(context).parkDialogParkOutButton),
    );
  }
}
