import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appDrawer.dart';
import 'package:parkingapp/util/utility.dart';

class EditVehicle extends StatelessWidget {
  static const String routeName = '/editVehicle';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Edit Vehicle'),
      ),
      //replace with back button
      drawer: AppDrawer(Routes.main),
      body: VehicleForm(),
    );
  }
}

class VehicleForm extends StatefulWidget {
  @override
  State<StatefulWidget> createState() => _VehicleFormState();
}

class _VehicleFormState extends State<VehicleForm> {
  //global key for form validation
  final _formKey = GlobalKey<FormState>();

  bool _vehicleChargeable = false, _parkNearExit = false, _parkingCard = false;
  String _name, _licensePlate;

  @override
  Widget build(BuildContext context) {
    return Form(
      key: _formKey,
      child: Padding(
        padding: EdgeInsets.all(10),
        child: Column(
          children: [
            TextFormField(
              autocorrect: false,
              decoration: InputDecoration(labelText: 'Fahrzeugname'),
              validator: (str) => requiredValue(str),
              onSaved: (str) => _name = str,
            ),
            TextFormField(
              autocorrect: false,
              decoration: InputDecoration(labelText: 'KFZ-Kennzeichen'),
              validator: (str) => requiredValue(str),
              onSaved: (str) => _licensePlate = str,
              inputFormatters: [UpperCaseTextFormatter()],
            ),
            //when this is toggled add the electric toggles
            SwitchListTile(
              title: Text('Fahrzeug ist ladefähig'),
              onChanged: (bool newValue) =>
                  setState(() => _vehicleChargeable = newValue),
              value: _vehicleChargeable,
            ),
            //generic toggles for all vehicles
            Divider(),
            SwitchListTile(
              title: Text('Nah am Ausgang Parken'),
              onChanged: (bool newValue) =>
                  setState(() => _parkNearExit = newValue),
              value: _parkNearExit,
            ),
            Divider(),
            SwitchListTile(
              title: Text('Parkkarte'),
              onChanged: (bool newValue) =>
                  setState(() => _parkingCard = newValue),
              value: _parkingCard,
            ),
            //end of form
            RaisedButton(
              child: Text('Fahrzeug hinzufügen'),
              onPressed: () => onPressed(),
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
  void onPressed() {
    var form = _formKey.currentState;
    if (form.validate()) {
      form.save();

      // create the vehicle that shall be added to the database
      Vehicle vehicle;
      if (_vehicleChargeable) {
        vehicle = LoadableVehicle(
            Utility.generateKey(),
            _name,
            _licensePlate,
            null,
            null,
            null,
            null,
            _parkNearExit,
            _parkingCard,
            null,
            null,
            null,
            null,
            null);
        /*
       --- handly with VehiclesDimensionDialog 
      this.height,
      this.width,
      this.length,
      this.turningCycle,
      ----
      this.nearExitPreference,
      this.parkingCard,
      --- Electric specific
      Extra toggle that allows for charge by default: this.doCharge,
      this.chargingProvider,
      this.chargeTimeBegin,
      this.chargeTimeEnd,
      this.charge*/
      } else {
        vehicle = StandardVehicle(Utility.generateKey(), _name, _licensePlate,
            null, null, null, null, _parkNearExit, _parkingCard);
        /*this.inAppKey,
      this.name,
      this.licensePlate,

      this.height,
      this.width,
      this.length,
      this.turningCycle,

      this.nearExitPreference,
      this.parkingCard*/
      }
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
