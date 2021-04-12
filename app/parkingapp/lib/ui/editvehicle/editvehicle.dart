import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:parkingapp/bloc/resources/subtitleformatter.dart';
import 'package:parkingapp/dialogs/chargetimedialog.dart';
import 'package:parkingapp/dialogs/chargingproviderdialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/dialogs/vehicledimensionsdialog.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/ui/FirstStart/routelandingpage.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/util/vehiclehelper.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// Edit an existing [Vehicle]
class EditVehicle extends StatelessWidget {
  final Vehicle vehicle;
  const EditVehicle({Key key, this.vehicle}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    //update vehicle in MainPage
    VehicleHelper.updateMainPageVehicle(
        context: context, parseVehicle: this.vehicle);
    return WillPopScope(
      onWillPop: () {
        return VehicleHelper.cleanUpDummy(
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

/// Create a new [Vehicle]
class CreateVehicle extends StatelessWidget {
  static const String routeName = '/createVehicle';

  @override
  Widget build(BuildContext context) {
    //update vehicle in MainPage
    VehicleHelper.updateMainPageVehicle(context: context);
    return WillPopScope(
      onWillPop: () => VehicleHelper.cleanUpDummy(context: context),
      child: Scaffold(
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).addVehicleTitle),
        ),
        body: VehicleForm(),
      ),
    );
  }
}

/// Display a Form to create a new [Vehicle] or update an existing [Vehicle]
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
  void initState() {
    //set vehicleDoCharge
    if (widget.vehicle.runtimeType == ChargeableVehicle) {
      ChargeableVehicle tempVehicle = vehicle;
      _vehicleDoCharge = tempVehicle.doCharge;
    }
    super.initState();
  }

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
                      subtitle: SubtitleFormatter.vehicleParkPreferences(
                        context: context,
                        vehicle: vehicle,
                      ),
                      onTap: () =>
                          _showDialog(context, ParkPreferencesDialog())),
                  Divider(),
                  FormField(
                    builder: (FormFieldState<dynamic> field) {
                      return ListTile(
                        title: Text(AppLocalizations.of(context)
                            .vehicleDimensionsDialogTitle),
                        subtitle: field.hasError
                            ? Text(
                                field.errorText,
                                style: TextStyle(
                                    color: Theme.of(context).errorColor),
                              )
                            : _vehicleDimensionsSubtitle(),
                        onTap: () =>
                            _showDialog(context, VehicleDimensionsDialog())
                                .then((value) => field.validate()),
                      );
                    },
                    validator: (value) => [
                      vehicle.length,
                      vehicle.width,
                      vehicle.height,
                      vehicle.turningCycle
                    ].every((value) => value == notSpecifiedDouble)
                        ? AppLocalizations.of(context).requiredText
                        : null,
                  )
                ],
              ),
            ),
            //end of form
            ElevatedButton(
              child: widget.vehicle == null
                  ? Text(AppLocalizations.of(context).addVehicleButton)
                  : Text(AppLocalizations.of(context).editVehicleButton),
              onPressed: () => validate(_vehicleChargeable),
            )
          ],
        ),
      ),
    );
  }

  /// show a dialog, wait for it to finish and update state after finishing
  /// returns true when finished
  Future<bool> _showDialog(BuildContext context, Widget dialog) async {
    await showDialog(context: context, builder: (context) => dialog);
    setState(() {});
    return true;
  }

  /// Subtitle for the [Vehicle] dimensions [FormField]
  Widget _vehicleDimensionsSubtitle() {
    if (vehicle.height == notSpecifiedDouble &&
        vehicle.width == notSpecifiedDouble &&
        vehicle.height == notSpecifiedDouble &&
        vehicle.turningCycle == notSpecifiedDouble) return null;
    return SubtitleFormatter.vehicleDimensions(
      context: context,
      vehicle: vehicle,
    );
  }

  /// [ChargeableVehicle] specific [ListTile]s
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

  /// [TextFormField] validator for required [String]s
  /// returns a string if no text is provided, otherwise null
  /// if null is returned the validator accepts the string
  String requiredValue(String str) {
    return str.isEmpty ? AppLocalizations.of(context).requiredText : null;
  }

  /// Validate the [VehicleForm] and save the [Vehicle]
  void validate(bool vehicleChargeable) async {
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
      } else {
        //sync vehicleDoCharge from toggle to vehicle
        ChargeableVehicle tempVehicle = vehicle;
        tempVehicle.doCharge = _vehicleDoCharge;
        vehicle = tempVehicle;
      }
      // create/update the vehicle
      //this will only "update" the vehicle because it has been created at the start
      DataHelper.addVehicle(context, vehicle);
      form.reset();
      SharedPreferences prefs = await SharedPreferences.getInstance();
      prefs.setBool(RouteLandingPage.isSetUp, true);
      //navigate to supplied route or pop the page of no route was supplied
      widget.route != null
          ? Navigator.of(context).pushReplacement(widget.route)
          : Navigator.of(context).pop();
    }
  }
}

/// Format text to uppercase in a [TextFormField]
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
