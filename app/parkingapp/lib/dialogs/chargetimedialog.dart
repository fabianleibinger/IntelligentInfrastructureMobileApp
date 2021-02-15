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

  TimeOfDay _chargeTimeBegin;
  TimeOfDay _chargeTimeEnd;
  TimeOfDay _picked;

  //sets the initial values
  @override
  void initState() {
    //TODO implement
    super.initState();
    _chargeAllDay = false;
    _chargeTimeBegin = TimeOfDay.now();
    _chargeTimeEnd = TimeOfDay.now();
  }

  //switches current vehicles value and listTile values
  void _setSwitchListTileValue(bool value) {
    setState(() {
      _chargeAllDay = value;
      if (value) {
        _chargeTimeBegin = TimeOfDay(hour: 00, minute: 00);
        _chargeTimeEnd = TimeOfDay(hour: 00, minute: 00);
      }
    });
  }

  //selects charge time begin in vehicle and listTile
  void _selectChargeTimeBegin(BuildContext context) async {
    _picked =
    await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    setState(() {
      _chargeTimeBegin = _picked;
      _checkHoleDay();
    });
  }

  //selects charge time end in vehicle and listTile
  void _selectChargeTimeEnd(BuildContext context) async {
    _picked =
    await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    setState(() {
      _chargeTimeEnd = _picked;
      _checkHoleDay();
    });
  }

  @override
  Widget build(BuildContext context) {
    return Constants.getConfirmationDialog(
        context,
        AppLocalizations
            .of(context)
            .chargeTimeDialogTitle,
        AppLocalizations
            .of(context)
            .settingsSaveButton,
        _getBody(context));
  }

  //returns one switch list tile and two clock widgets
  _getBody(BuildContext context) {
    return Column(
      children: [
        SwitchListTile(
            title: Text(AppLocalizations
                .of(context)
                .chargeTimeDialogSwitchListTileTitle),
            subtitle: Text(AppLocalizations
                .of(context)
                .chargeTimeDialogSwitchListTileSubtitle),
            value: _chargeAllDay,
            onChanged: (value) {
              _setSwitchListTileValue(value);
            }),
        Divider(),
        ListTile(
            title: Text(AppLocalizations
                .of(context)
                .chargeTimeDialogChargeTimeBeginTitle),
            subtitle: Text(_chargeTimeBegin.format(context)),
            onTap: () {
              _selectChargeTimeBegin(context);
            }),
        ListTile(
            title: Text(AppLocalizations
                .of(context)
                .chargeTimeDialogChargeTimeEndTitle),
            subtitle: Text(_chargeTimeEnd.format(context)),
            onTap: () {
              _selectChargeTimeEnd(context);
            })
      ],
    );
  }

  //checks if switchListTile for charging hole day has to be true or false
  void _checkHoleDay() {
    if (_chargeTimeBegin == TimeOfDay(hour: 00, minute: 00) &&
        _chargeTimeEnd == TimeOfDay(hour: 00, minute: 00)) {
      _setSwitchListTileValue(true);
    } else {
      _setSwitchListTileValue(false);
    }
  }
}
