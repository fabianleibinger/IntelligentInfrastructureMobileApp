import 'package:app_settings/app_settings.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'constants.dart';

/// The dialog that tells the user that no connection can be established at the moment.
///
/// ```showDialog(
///             context: context,
///             builder: (context) {
///               return NoConnectionDialog.getDialog(context);
///             });
/// ```
class NoConnectionDialog {
  static const double _dividerThickness = 0;

  /// Returns dialog that can open WIFI settings
  /// and opens the correct page for the vehicle.
  static getDialog(BuildContext context) {
    return AlertDialog(
      title: Text(AppLocalizations.of(context).noConnectionDialogTitle),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text(AppLocalizations.of(context).noConnectionDialogContent),
          Divider(thickness: _dividerThickness),
          ElevatedButton(
              onPressed: AppSettings.openWIFISettings,
              child: Text(AppLocalizations.of(context).networkButton))
        ],
      ),
      actions: [
        Constants.getTextButton(
            context,
            red,
            AppLocalizations.of(context).parkDialogBackButton,
            Routes.returnCorrectRouteForVehicle(vehicle))
      ],
    );
  }
}
