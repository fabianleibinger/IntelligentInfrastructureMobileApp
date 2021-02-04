import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class ParkDialog {
  static createDialog(BuildContext context) {
    return showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            backgroundColor: white,
            title: Text(AppLocalizations.of(context).parkDialogHeader),
            content: Text(AppLocalizations.of(context).parkDialogBody),
            actions: [
              FlatButton(
                textColor: red,
                onPressed: () {Navigator.pop(context);},
                child: Text(AppLocalizations.of(context).parkDialogCancelButton),
              ),
              FlatButton(
                textColor: green,
                onPressed: () {},
                child: Text(AppLocalizations.of(context).parkDialogParkInButton),
              ),
            ],
          );
        });
  }
}
