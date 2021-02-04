import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/constants.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/routes/routes.dart';

class ParkDialog {
  static createParkInDialog(BuildContext context) {
    return Constants.createParkAlertDialog(
        context,
        AppLocalizations.of(context).parkDialogParkInTitle,
        AppLocalizations.of(context).parkDialogParkInContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkInButton,
        Routes.park);
  }

  static createParkOutDialog(BuildContext context) {
    return Constants.createParkAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkOutContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        //TODO Seite anpassen
        Routes.park);
  }

  static createParkCancelDialog(BuildContext context) {
    return Constants.createParkAlertDialogNoTitle(
        context,
        AppLocalizations.of(context).parkDialogParkCancelContent,
        AppLocalizations.of(context).parkDialogBackButton,
        AppLocalizations.of(context).parkDialogParkOutButton,
        //TODO Seite anpassen
        Routes.park);
  }
}
