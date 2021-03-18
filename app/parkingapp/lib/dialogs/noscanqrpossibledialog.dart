import 'package:flutter/cupertino.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'constants.dart';

//defines dialog for no scan possible situations
class NoScanPossibleDialog {
  //returns the dialog for vehicles currently parking in or out
  static getVehicleMovingDialog(BuildContext context) {
    return Constants.getAlertDialogOneBackButtonNoTitle(
        context,
        AppLocalizations.of(context).noScanPossibleDialogText,
        AppLocalizations.of(context).parkDialogBackButton);
  }
}
