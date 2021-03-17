import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/settingspage/qrpage.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//defines the qr code dialog
class ScanQRDialog {

  //returns qr dialog for specific vehicle
  static getVehicleQRDialog(BuildContext context, Vehicle vehicle) {
    return AlertDialog(
      title: Text(AppLocalizations.of(context).scanQRDialogTitle),
      content: Text(AppLocalizations.of(context).scanQRDialogContent),
      actions: [
        Constants.createBackFlatButton(context, red,
            AppLocalizations.of(context).scanQRDialogCancelButton),
        FlatButton(
          textColor: green,
          onPressed: () => Navigator.push(
              context,
              MaterialPageRoute(
                  builder: (BuildContext context) => QRPage(vehicle: vehicle))),
          child: Text(AppLocalizations.of(context).scanQRDialogConfirmButton),
        )
      ],
    );
  }
}
