import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:parkingapp/dialogs/chargetimedialog.dart';
import 'package:parkingapp/dialogs/chargingproviderdialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/dialogs/vehicledimensionsdialog.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/enum/chargingprovider.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/util/utility.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

final double _notSpecifiedDouble = 0;
final String _notSpecifiedString = '';
final bool _notSpecifiedBool = false;
final TimeOfDay _notSpecifiedTimeOfDay = TimeOfDay(hour: 0, minute: 0);
final String _defaultChargingProvider =
    ChargingProvider.Automatisch.toShortString();

class EditVehicle extends StatelessWidget {
  final Vehicle vehicle;
  const EditVehicle({Key key, this.vehicle}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    vehicle != null
        ? print('Update vehicle: ' +
            vehicle.inAppKey +
            ', ' +
            vehicle.name +
            ', ' +
            vehicle.licensePlate)
        : null;

    //update vehicle in MainPage
    UpdateMainPageVehicle.setUp(context: context, parseVehicle: this.vehicle);
    return WillPopScope(
      onWillPop: () {
        return UpdateMainPageVehicle.cleanUp(
            context: context, parseVehicle: this.vehicle);
      },
      child: Scaffold(
        appBar: AppBar(
          automaticallyImplyLeading: false,
          title: Text(AppLocalizations.of(context).editVehicleTitle),
        ),
        body: VehicleForm(
          vehicle: vehicle,
        ),
      ),
    );
  }
}

class CreateVehicle extends StatelessWidget {
  static const String routeName = '/createVehicle';

  @override
  Widget build(BuildContext context) {
    //update vehicle in MainPage
    UpdateMainPageVehicle.setUp(context: context);
    return WillPopScope(
      onWillPop: () => UpdateMainPageVehicle.cleanUp(context: context),
      child: Scaffold(
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).addVehicleTitle),
        ),
        body: VehicleForm(),
      ),
    );
  }
}

class VehicleForm extends StatefulWidget {
  final Vehicle vehicle;
  //this route will be called when the form is completed
  final MaterialPageRoute route;
  const VehicleForm({Key key, this.vehicle, this.route}) : super(key: key);
  @override
  State<StatefulWidget> createState() => _VehicleFormState();
}

class _VehicleFormState extends State<VehicleForm> {
  //global key for form validation
  final _formKey = GlobalKey<FormState>();

  //local variables for states
  bool _vehicleChargeable;
  //vehicle specific variables (these should be saved in the vehicle)
  bool _vehicleDoCharge = false;

  @override
  Widget build(BuildContext context) {
    //preselect if vehicle is chargeable or not
    if (_vehicleChargeable == null)
      _vehicleChargeable = widget.vehicle.runtimeType == ChargeableVehicle;
    return Form(
      key: _formKey,
      child: Padding(
        padding: EdgeInsets.all(10),
        child: Column(
          children: [
            Expanded(
              child: ListView(
                children: [
                  TextFormField(
                    autocorrect: false,
                    decoration: InputDecoration(
                        labelText: AppLocalizations.of(context).vehicleName),
                    initialValue: vehicle.name,
                    validator: (str) => requiredValue(str),
                    onSaved: (str) => vehicle.name = str,
                  ),
                  TextFormField(
                    autocorrect: false,
                    decoration: InputDecoration(
                        labelText: AppLocalizations.of(context).licensePlate),
                    initialValue: vehicle.licensePlate,
                    validator: (str) => requiredValue(str),
                    onSaved: (str) => vehicle.licensePlate = str,
                    inputFormatters: [UpperCaseTextFormatter()],
                  ),
                  //when this is toggled add the electric toggles
                  //TODO animate expand
                  //TODO move this into a seperate Form
                  SwitchListTile(
                    title: Text(AppLocalizations.of(context).vehicleCanCharge),
                    onChanged: (bool newValue) =>
                        setState(() => _vehicleChargeable = newValue),
                    value: _vehicleChargeable,
                  ),
                  _getElectricToggles(_vehicleChargeable),
                  //generic toggles for all vehicles
                  Divider(),
                  ListTile(
                      title: Text(AppLocalizations.of(context).parkPreferences),
                      subtitle: _vehicleParkPreferencesSubtitle(context),
                      onTap: () =>
                          _showDialog(context, ParkPreferencesDialog())),
                  Divider(),
                  ListTile(
                    title: Text(AppLocalizations.of(context)
                        .vehicleDimensionsDialogTitle),
                    subtitle: _vehicleDimensionsSubtitle(context),
                    onTap: () =>
                        _showDialog(context, VehicleDimensionsDialog()),
                  ),
                ],
              ),
            ),
            //end of form
            RaisedButton(
              child: widget.vehicle == null
                  ? Text(AppLocalizations.of(context).addVehicleButton)
                  : Text(AppLocalizations.of(context).editVehicleButton),
              onPressed: () => onPressed(_vehicleChargeable),
              highlightColor: Theme.of(context).accentColor,
              color: Theme.of(context).primaryColor,
              colorBrightness: Theme.of(context).primaryColorBrightness,
            )
          ],
        ),
      ),
    );
  }

  // TODO merge into one funciton
  // select Time (used for start and end of charge)
  void _showDialog(BuildContext context, Widget dialog) async {
    await showDialog(context: context, builder: (context) => dialog);
    setState(() {});
  }

  //electric vehicle toggles
  Column _getElectricToggles(bool vehicleChargeable) {
    //return no toggles if not chargeable
    if (!vehicleChargeable) return Column();
    //return toggles if chargeable
    ChargeableVehicle tempVehicle = vehicle;
    List<Widget> _electricToggles = [];
    _electricToggles.addAll([
      Divider(),
      SwitchListTile(
        title: Text(AppLocalizations.of(context).chargeVehicleByDefault),
        onChanged: (bool newValue) =>
            setState(() => _vehicleDoCharge = newValue),
        value: _vehicleDoCharge,
      ),
      Divider(),
      ListTile(
        title: Text(AppLocalizations.of(context).chargingProviderDialogTitle),
        subtitle: Text(tempVehicle.chargingProvider),
        onTap: () => _showDialog(context, ChargingProviderDialog()),
      ),
      Divider(),
      ListTile(
        title: Text(AppLocalizations.of(context).chargeTimeDialogTitle),
        subtitle: Text(AppLocalizations.of(context).begin +
            AppLocalizations.of(context).colonSpace +
            tempVehicle.chargeTimeBegin.format(context) +
            AppLocalizations.of(context).space +
            AppLocalizations.of(context).end +
            AppLocalizations.of(context).colonSpace +
            tempVehicle.chargeTimeEnd.format(context)),
        onTap: () => _showDialog(context, ChargeTimeDialog()),
      )
    ]);
    return Column(
      children: _electricToggles,
    );
  }

  // returns a string if no text is provided, otherwise null
  // if null is returned the validator accepts the string
  String requiredValue(String str) {
    return str.isEmpty ? AppLocalizations.of(context).requiredText : null;
  }

  //validate the form
  void onPressed(bool vehicleChargeable) {
    var form = _formKey.currentState;
    if (form.validate()) {
      form.save();

      //update does not work if the runtime type changes
      DataHelper.deleteVehicle(context, vehicle);
      //convert vehicle to standardvehicle if necessary
      if (!vehicleChargeable) {
        print('converting to standard vehicle');
        ChargeableVehicle convertVehicle = vehicle;
        vehicle = convertVehicle.toStandardVehicle();
      }
      // create/update the vehicle
      //this will only "update" the vehicle because it has been created at the start
      DataHelper.addVehicle(context, vehicle);
      form.reset();
      //TODO move to the Scaffold Widget from EditVehicle/AddVehicle
      //navigate to supplied route or pop the page of no route was supplied
      widget.route != null
          ? Navigator.of(context).pushReplacement(widget.route)
          : Navigator.of(context).pop();
    }
  }

  Text _vehicleDimensionsSubtitle(BuildContext context) {
    if (vehicle.height == _notSpecifiedDouble &&
        vehicle.width == _notSpecifiedDouble &&
        vehicle.height == _notSpecifiedDouble &&
        vehicle.turningCycle == _notSpecifiedDouble) return null;
    return Text(AppLocalizations.of(context).length +
        AppLocalizations.of(context).colonSpace +
        (vehicle.length / 1000).toStringAsPrecision(3) +
        AppLocalizations.of(context).meterShort +
        AppLocalizations.of(context).space +
        AppLocalizations.of(context).width +
        AppLocalizations.of(context).colonSpace +
        (vehicle.width / 1000).toStringAsPrecision(3) +
        AppLocalizations.of(context).meterShort +
        AppLocalizations.of(context).space +
        AppLocalizations.of(context).height +
        AppLocalizations.of(context).colonSpace +
        (vehicle.height / 1000).toStringAsPrecision(3) +
        AppLocalizations.of(context).meterShort);
  }

  Text _vehicleParkPreferencesSubtitle(BuildContext context) {
    StringBuffer text = StringBuffer();
    if (vehicle.parkingCard) {
      text.write(AppLocalizations.of(context).parkingCard);
      text.write(AppLocalizations.of(context).commaSeperatedList);
    }
    if (vehicle.nearExitPreference) {
      text.write(AppLocalizations.of(context).nearExitPreference);
      text.write(AppLocalizations.of(context).commaSeperatedList);
    }
    if (text.length >= AppLocalizations.of(context).commaSeperatedList.length)
      // remove the last comma and return
      return Text(text.toString().substring(
          0,
          text.length -
              AppLocalizations.of(context).commaSeperatedList.length));
    else
      return null;
  }
}

//format text to uppercase
class UpperCaseTextFormatter extends TextInputFormatter {
  @override
  TextEditingValue formatEditUpdate(
      TextEditingValue oldValue, TextEditingValue newValue) {
    return TextEditingValue(
      text: newValue.text?.toUpperCase(),
      selection: newValue.selection,
    );
  }
}

class UpdateMainPageVehicle {
  //TODO maybe put setUp into a constructor of a specific element and destroying this element with the cleanUp method. This would mean not specifying the parseVehicle to the cleanUp
  static void setUp({@required BuildContext context, Vehicle parseVehicle}) {
    if (parseVehicle == null) {
      print('set new electric vehicle on main page');
      vehicle = ChargeableVehicle(
        Utility.generateKey(),
        _notSpecifiedString,
        _notSpecifiedString,
        _notSpecifiedDouble,
        _notSpecifiedDouble,
        _notSpecifiedDouble,
        _notSpecifiedDouble,
        _notSpecifiedBool,
        _notSpecifiedBool,
        _notSpecifiedBool,
        _notSpecifiedBool,
        _defaultChargingProvider,
        _notSpecifiedTimeOfDay,
        _notSpecifiedTimeOfDay,
      );
      print('adding dummy vehicle to database');
      DataHelper.addVehicle(context, vehicle);
    } else if (parseVehicle.runtimeType == ChargeableVehicle) {
      print('parsedVehicle is electric; using parsed vehicle as is');
      vehicle = parseVehicle;
    } else if (parseVehicle.runtimeType == StandardVehicle) {
      DataHelper.deleteVehicle(context, vehicle);
      print('parsedVehicle is standard; converting into electric');
      StandardVehicle convertVehicle = parseVehicle;
      vehicle = convertVehicle.toElectricVehicle();
      print('converting standard vehicle to electric vehicle in database');
      DataHelper.addVehicle(context, vehicle);
    }
  }

  static Future<bool> cleanUp(
      {@required BuildContext context, Vehicle parseVehicle}) async {
    print('starting clean up');
    if (parseVehicle == null) {
      print('removing dummy vehicle');
      //the behicle did not exist before opening the form and needs to be removed
      DataHelper.deleteVehicle(context, vehicle);
    } else {
      print('restoring previous state');
      print('previous vehicle: inAppID: ' +
          parseVehicle.inAppKey +
          ' name: ' +
          parseVehicle.name +
          ' licensePlate: ' +
          parseVehicle.licensePlate +
          ' parkpreferences: ' +
          parseVehicle.nearExitPreference.toString() +
          parseVehicle.parkingCard.toString());
      print('temporary vehicle: inAppID: ' +
          vehicle.inAppKey +
          ' name: ' +
          vehicle.name +
          ' licensePlate: ' +
          vehicle.licensePlate +
          ' parkpreferences: ' +
          vehicle.nearExitPreference.toString() +
          vehicle.parkingCard.toString());
      //restore the previous state
      DataHelper.updateVehicle(context, parseVehicle);
    }
    return true;
  }
}
