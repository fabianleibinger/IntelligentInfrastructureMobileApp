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

  //switches current vehicles value and listTile value
  void _setSwitchListTileValue(bool value) {
    setState(() {
      _chargeAllDay = value;
    });
  }

  void _selectTime(BuildContext context, TimeOfDay time) async {
    _picked =
        await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    setState(() {
      time = _picked;
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
        ListTile(
            title: Text('Startzeit'),
            subtitle: Text(_chargeTimeBegin.format(context)),
            onTap: () {
              _selectTime(context, _chargeTimeBegin);
            }),
        ListTile(
            title: Text('Endzeit'),
            subtitle: Text(_chargeTimeEnd.format(context)),
            onTap: () {
              _selectTime(context, _chargeTimeEnd);
            })
      ],
    );
  }
}
