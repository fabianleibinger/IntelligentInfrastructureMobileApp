import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class ConfirmDelete extends AlertDialog {
  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(AppLocalizations.of(context).askDeleteVehicle),
      content: Text(AppLocalizations.of(context).askDeleteVehicleContent),
      actions: [
        FlatButton(
          child: Text(AppLocalizations.of(context).scanQRDialogCancelButton),
          onPressed: () => Navigator.of(context).pop(false),
        ),
        FlatButton(
          child: Text(AppLocalizations.of(context).scanQRDialogConfirmButton),
          onPressed: () => Navigator.of(context).pop(true),
        )
      ],
    );
  }
}

class CantDeleteVehicle extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(AppLocalizations.of(context).vehicleIsParkedInTitle),
      content: Text(AppLocalizations.of(context).vehicleIsParkedInBody),
      actions: [
        FlatButton(
          child: Text(AppLocalizations.of(context).buttonOk),
          onPressed: () => Navigator.of(context).pop(false),
        )
      ],
    );
  }
}
