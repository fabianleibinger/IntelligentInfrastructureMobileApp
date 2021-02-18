import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/dialogs/chargetimedialog.dart';
import 'package:parkingapp/dialogs/chargingproviderdialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/dialogs/vehicledimensionsdialog.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/util/utility.dart';

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
    _UpdateMainPageVehicle(parseVehicle: vehicle);
    return Scaffold(
      appBar: AppBar(
        title: Text('Edit Vehicle'),
      ),
      body: VehicleForm(
        vehicle: vehicle,
      ),
    );
  }
}

class CreateVehicle extends StatelessWidget {
  static const String routeName = '/createVehicle';

  @override
  Widget build(BuildContext context) {
    //update vehicle in MainPage
    _UpdateMainPageVehicle();
    return Scaffold(
      appBar: AppBar(
        title: Text('Add Vehicle'),
      ),
      body: VehicleForm(),
    );
  }
}

class VehicleForm extends StatefulWidget {
  final Vehicle vehicle;
  const VehicleForm({Key key, this.vehicle}) : super(key: key);
  @override
  State<StatefulWidget> createState() => _VehicleFormState();
}

class _VehicleFormState extends State<VehicleForm> {
  //global key for form validation
  final _formKey = GlobalKey<FormState>();

  //local variables for states
  bool _vehicleChargeable = false;
  List<Widget> _electricToggles = [];
  //vehicle specific variables (these should be saved in the vehicle)
  bool _vehicleDoCharge = false;

  @override
  Widget build(BuildContext context) {
    return Form(
      key: _formKey,
      child: Padding(
        padding: EdgeInsets.all(10),
        child: ListView(
          children: [
            TextFormField(
              autocorrect: false,
              decoration: InputDecoration(labelText: 'Fahrzeugname'),
              validator: (str) => requiredValue(str),
              onSaved: (str) => vehicle.name = str,
            ),
            TextFormField(
              autocorrect: false,
              decoration: InputDecoration(labelText: 'KFZ-Kennzeichen'),
              validator: (str) => requiredValue(str),
              onSaved: (str) => vehicle.licensePlate = str,
              inputFormatters: [UpperCaseTextFormatter()],
            ),
            //when this is toggled add the electric toggles
            //TODO animate expand
            //TODO move this into a seperate Form
            SwitchListTile(
              title: Text('Fahrzeug ist ladefähig'),
              onChanged: (bool newValue) => setState(() {
                _vehicleChargeable = newValue;
                if (_vehicleChargeable) {
                  _electricToggles.addAll([
                    Divider(),
                    SwitchListTile(
                      title: Text('Fahrzeug standardmäßig laden'),
                      onChanged: (bool newValue) =>
                          setState(() => _vehicleDoCharge = newValue),
                      value: _vehicleDoCharge,
                    ),
                    Divider(),
                    ListTile(
                      title: Text('Ladeanbieter'),
                      onTap: () => showDialog(
                          context: context,
                          builder: (context) => ChargingProviderDialog()),
                    ),
                    Divider(),
                    ListTile(
                      title: Text('Ladeuhrzeit'),
                      onTap: () => showDialog(
                          context: context,
                          builder: (context) => ChargeTimeDialog()),
                    )
                  ]);
                } else {
                  _electricToggles.clear();
                }
              }),
              value: _vehicleChargeable,
            ),
            Column(
              children: _electricToggles,
            ),
            //generic toggles for all vehicles
            Divider(),
            ListTile(
                title: Text('Parkpräferenzen'),
                onTap: () => showDialog(
                    context: context,
                    builder: (context) => ParkPreferencesDialog())),
            Divider(),
            ListTile(
              title: Text('Fahrzeugabmessungen'),
              onTap: () => showDialog(
                  context: context,
                  builder: (context) => VehicleDimensionsDialog()),
            ),
            //end of form
            RaisedButton(
              child: Text('Fahrzeug hinzufügen'),
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

  // returns a string if no text is provided, otherwise null
  // if null is returned the validator accepts the string
  String requiredValue(String str) {
    return str.isEmpty ? "Erforderlich" : null;
  }

  //validate the form
  void onPressed(bool vehicleChargeable) {
    var form = _formKey.currentState;
    if (form.validate()) {
      form.save();

      //convert vehicle to standardvehicle if necessary
      if (!vehicleChargeable) {
        LoadableVehicle convertVehicle = vehicle;
        vehicle = convertVehicle.toStandardVehicle();
      }
      // create/update the vehicle
      widget.vehicle == null
          ? DataHelper.addVehicle(context, vehicle)
          : DataHelper.updateVehicle(context, vehicle);
      form.reset();
      //TODO move to the Scaffold Widget from EditVehicle/AddVehicle
      Navigator.of(context).pop();
    }
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

class _UpdateMainPageVehicle {
  _UpdateMainPageVehicle({Vehicle parseVehicle}) {
    if (parseVehicle == null) {
      print('set new electric vehicle on main page');
      vehicle = LoadableVehicle(
          Utility.generateKey(),
          '',
          '',
          0,
          0,
          0,
          0,
          false,
          false,
          false,
          '',
          TimeOfDay(hour: 0, minute: 0),
          TimeOfDay(hour: 0, minute: 0),
          null);
    } else if (parseVehicle.runtimeType == LoadableVehicle) {
      print('parsedVehicle is electric; using parsed vehicle as is');
      vehicle = parseVehicle;
    } else if (parseVehicle.runtimeType == StandardVehicle) {
      print('parsedVehicle is standard; converting into electric');
      StandardVehicle convertVehicle = parseVehicle;
      vehicle = convertVehicle.toElectricVehicle();
    }
  }
}
