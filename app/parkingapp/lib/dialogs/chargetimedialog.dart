import 'package:flutter/material.dart';
import 'constants.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';

///The dialog that sets charge times for [ChargeableVehicles].
class ChargeTimeDialog extends StatefulWidget {
  ChargeTimeDialog({Key key}) : super(key: key);

  @override
  _ChargeTimeDialogState createState() => _ChargeTimeDialogState();
}

class _ChargeTimeDialogState extends State<ChargeTimeDialog> {
  static final double _dividerThickness = 1;

  static final TimeOfDay _midnight = TimeOfDay(hour: 00, minute: 00);

  /// The values of the dialog [Tiles].
  bool _chargeAllDay = true;
  TimeOfDay _chargeTimeBegin = _midnight;
  TimeOfDay _chargeTimeEnd = _midnight;

  /// The time picked by [showTimePicker]
  TimeOfDay _picked;

  /// Sets the initially displayed [Tiles] values.
  @override
  void initState() {
    super.initState();
    if (vehicle.runtimeType == ChargeableVehicle) {
      _checkChargeTimes(vehicle);
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

  /// Returns one [SwitchListTile] for hole day
  /// and two clock widgets [ListTile] for begin and end.
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

  /// Switches hole day [SwitchListTile] value and sets vehicle times accordingly.
  void _setSwitchListTileValue(bool value) {
    setState(() {
      _chargeAllDay = value;
      if (value) {
        _chargeTimeBegin = _midnight;
        _chargeTimeEnd = _midnight;
        if (vehicle.runtimeType == ChargeableVehicle) {
          _setChargeTimeVehicle(vehicle);
        }
      }
    });
  }

  /// Selects charge time begin and sets vehicle times
  /// and all displayed [Tiles] accordingly.
  void _selectChargeTimeBegin(BuildContext context) async {
    _picked =
        await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    if (_picked != null) {
      setState(() {
        _chargeTimeBegin = _picked;
        if (vehicle.runtimeType == ChargeableVehicle) {
          _setChargeTimeVehicle(vehicle);
          _checkHoleDay();
        }
      });
    }
  }

  /// Selects charge time end and sets vehicle times
  /// and all displayed [Tiles] accordingly.
  void _selectChargeTimeEnd(BuildContext context) async {
    _picked =
        await showTimePicker(context: context, initialTime: _chargeTimeBegin);
    if (_picked != null) {
      setState(() {
        _chargeTimeEnd = _picked;
        if (vehicle.runtimeType == ChargeableVehicle) {
          _setChargeTimeVehicle(vehicle);
          _checkHoleDay();
        }
      });
    }
  }

  /// Sets [vehicle] charge time values.
  void _setChargeTimeVehicle(ChargeableVehicle vehicle) {
    vehicle.setAndUpdateChargeTimeBegin(context, _chargeTimeBegin);
    vehicle.setAndUpdateChargeTimeEnd(context, _chargeTimeEnd);
  }

  /// Selects the correct charge time values of a chargeable [vehicle]
  /// for all displayed [Tiles].
  void _checkChargeTimes(ChargeableVehicle vehicle) {
    _chargeTimeBegin = vehicle.chargeTimeBegin;
    _chargeTimeEnd = vehicle.chargeTimeEnd;
    _checkHoleDay();
  }

  /// Checks if [SwitchListTile] for charging hole day should display true or false.
  void _checkHoleDay() {
    if (_chargeTimeBegin == _midnight && _chargeTimeEnd == _midnight) {
      _setSwitchListTileValue(true);
    } else {
      _setSwitchListTileValue(false);
    }
  }
}
