import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';

//defines the charge time dialog
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

  final TimeOfDay _midnight = TimeOfDay(hour: 00, minute: 00);
  final double _dividerThickness = 1;

  //sets the initial values
  @override
  void initState() {
    super.initState();
    if (vehicle.runtimeType == LoadableVehicle) {
      _checkChargeTimes(vehicle);
    }
  }

  //switches current vehicles value and listTile values
  void _setSwitchListTileValue(bool value) {
    setState(() {
      _chargeAllDay = value;
      if (value) {
        _chargeTimeBegin = _midnight;
        _chargeTimeEnd = _midnight;
        _setChargeTimeVehicle(vehicle);
      }
    });
  }

  //selects charge time begin in vehicle and listTile
  void _selectChargeTimeBegin(BuildContext context) async {
    _picked =
        await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    if (_picked != null) {
      setState(() {
        _chargeTimeBegin = _picked;
        _setChargeTimeVehicle(vehicle);
        _checkHoleDay();
      });
    }
  }

  //selects charge time end in vehicle and listTile
  void _selectChargeTimeEnd(BuildContext context) async {
    _picked =
        await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    if (_picked != null) {
      setState(() {
        _chargeTimeEnd = _picked;
        _setChargeTimeVehicle(vehicle);
        _checkHoleDay();
      });
    }
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
        Divider(thickness: _dividerThickness),
        ListTile(
            title: Text(AppLocalizations.of(context)
                .chargeTimeDialogChargeTimeBeginTitle),
            subtitle: Text(_chargeTimeBegin.format(context)),
            onTap: () {
              _selectChargeTimeBegin(context);
            }),
        ListTile(
            title: Text(AppLocalizations.of(context)
                .chargeTimeDialogChargeTimeEndTitle),
            subtitle: Text(_chargeTimeEnd.format(context)),
            onTap: () {
              _selectChargeTimeEnd(context);
            })
      ],
    );
  }

  //sets vehicles charge time values
  void _setChargeTimeVehicle(LoadableVehicle vehicle) {
    vehicle.chargeTimeBegin = _chargeTimeBegin;
    vehicle.chargeTimeEnd = _chargeTimeEnd;
    DatabaseProvider.db.update(vehicle);
  }

  //selects the right charge time values for all tiles
  void _checkChargeTimes(LoadableVehicle vehicle) {
    _chargeTimeBegin = vehicle.chargeTimeBegin;
    _chargeTimeEnd = vehicle.chargeTimeEnd;
    _checkHoleDay();
  }

  //checks if switchListTile for charging hole day has to be true or false
  void _checkHoleDay() {
    if (_chargeTimeBegin == _midnight && _chargeTimeEnd == _midnight) {
      _setSwitchListTileValue(true);
    } else {
      _setSwitchListTileValue(false);
    }
  }
}
