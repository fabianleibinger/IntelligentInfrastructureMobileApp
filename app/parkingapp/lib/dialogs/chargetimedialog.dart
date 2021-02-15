import 'package:flutter/material.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class ChargeTimeDialog extends StatefulWidget {
  ChargeTimeDialog({Key key}) : super(key: key);

  @override
  _ChargeTimeDialogState createState() => _ChargeTimeDialogState();
}

class _ChargeTimeDialogState extends State<ChargeTimeDialog> {
  bool _chargeAllDay;

  //sets the initial values
  @override
  void initState() {
    // TODO: implement initState
    super.initState();
    _chargeAllDay = false;
  }

  //switches current vehicles value and listTile value
  void _setSwitchListTileValue(bool value) {
    setState(() {
      _chargeAllDay = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations.of(context).chargeTimeDialogTitle,
        AppLocalizations.of(context).settingsSaveButton,
        _getBody(context));
  }

  //returns one switch list tile and two clock widgets
  _getBody(BuildContext context) {
    return Column(
      children: [
        SwitchListTile(
            title: Text(AppLocalizations.of(context)
                .chargeTimeDialogSwitchListTileTitle),
            subtitle: Text(AppLocalizations.of(context)
                .chargeTimeDialogSwitchListTileSubtitle),
            value: _chargeAllDay,
            onChanged: (value) {
              _setSwitchListTileValue(value);
            }),
        Divider(),

      ],
    );
  }
}
