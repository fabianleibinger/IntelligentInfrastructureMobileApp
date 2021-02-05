import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'constants.dart';

class ChargingProviderDialog {
  static createChargingProviderDialog(BuildContext context) {
    return Constants.createConfirmationDialog(context, 'title', 'button', Constants.listTiles(context));
  }
}
