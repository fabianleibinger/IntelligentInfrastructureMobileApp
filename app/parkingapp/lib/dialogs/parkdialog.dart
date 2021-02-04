import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/constants.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/routes/routes.dart';

class ParkDialog {
  static createParkInDialog(BuildContext context) {
    return Constants.createAlertDialog(
        context,
        AppLocalizations.of(context).parkDialogTitle,
        AppLocalizations.of(context).parkDialogContent,
        AppLocalizations.of(context).parkDialogCancelButton,
        AppLocalizations.of(context).parkDialogParkInButton,
        Routes.park);
  }
}
