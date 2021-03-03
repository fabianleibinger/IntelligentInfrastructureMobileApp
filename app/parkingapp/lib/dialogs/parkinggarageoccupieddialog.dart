import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

//defines the dialog that tells the user that the parking garage is occupied
//button leads back to MainPage
class ParkingGarageOccupiedDialog {
  static createDialog(BuildContext context) {
    return Constants.createAlertDialogOneButtonNoTitle(
        context,
        AppLocalizations.of(context).parkingGarageOccupiedDialogContent,
        AppLocalizations.of(context).parkDialogBackButton,
        vehicle.inAppKey);
  }
}
