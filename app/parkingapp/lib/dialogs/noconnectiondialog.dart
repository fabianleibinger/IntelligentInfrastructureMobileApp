import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

//defines the dialog that tells the user that no connection can be established at the moment
//button leads back to MainPage
class NoConnectionDialog {
  static createDialog(BuildContext context) {
    return Constants.createAlertDialogOneButtonNoTitle(
        context,
        AppLocalizations.of(context).noConnectionDialogContent,
        AppLocalizations.of(context).parkDialogBackButton,
        vehicle.inAppKey);
  }
}
