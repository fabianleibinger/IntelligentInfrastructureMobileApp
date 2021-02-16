import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
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
    return Scaffold(
      appBar: AppBar(
        title: Text('Edit Vehicle'),
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

  bool _vehicleChargeable = false,
      _parkNearExit = false,
      _parkingCard = false,
      _vehicleDoCharge = true;
  String _name, _licensePlate, _chargingProvider;
  TimeOfDay _chargeBegin = TimeOfDay(hour: 0, minute: 0),
      _chargeEnd = TimeOfDay(hour: 23, minute: 59);
  List<Widget> _electricToggles = [];

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
                      title: Text('Charge Time Begin'),
                      onTap: () => _selectChargeBeginTime(context),
                    ),
                    Divider(),
                    ListTile(
                      title: Text('Charge End Time'),
                      onTap: () => _selectChargeEndTime(context),
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

  // TODO merge into one funciton
  // select Time (used for start and end of charge)
  void _selectChargeBeginTime(BuildContext context) async {
    TimeOfDay timeOfDay = await showTimePicker(
        context: context,
        initialTime: _chargeBegin,
        helpText: 'Fahrzeug bevorzugt laden ab: ');
    setState(() => _chargeBegin = timeOfDay);
  }

  // select Time (used for start and end of charge)
  void _selectChargeEndTime(BuildContext context) async {
    TimeOfDay timeOfDay = await showTimePicker(
        context: context,
        initialTime: _chargeEnd,
        helpText: 'Fahrzeug bevorzugt laden ab: ');
    setState(() => _chargeEnd = timeOfDay);
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
      Vehicle vehicle = widget.vehicle;
      if (_vehicleChargeable) {
        vehicle = LoadableVehicle(
            Utility.generateKey(),
            _name,
            _licensePlate,
            0,
            0,
            0,
            0,
            _parkNearExit,
            _parkingCard,
            _vehicleDoCharge,
            _chargingProvider,
            DateTime(DateTime.now().year, DateTime.now().month,
                DateTime.now().day, _chargeBegin.hour, _chargeBegin.minute),
            DateTime(DateTime.now().year, DateTime.now().month,
                DateTime.now().day, _chargeEnd.hour, _chargeEnd.minute),
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
            0, 0, 0, 0, _parkNearExit, _parkingCard);
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

      //TODO update vehicle if necessary
      //addd vehicle to database
      DatabaseProvider.db.insert(vehicle).then((vehicle) =>
          BlocProvider.of<VehicleBloc>(context).add(AddVehicle(vehicle)));
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
