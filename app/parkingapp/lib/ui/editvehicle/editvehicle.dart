import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appDrawer.dart';

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

  bool _vehicleChargeable = false;
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
              title: Text('chargeable'),
              onChanged: (bool newValue) =>
                  setState(() => _vehicleChargeable = newValue),
              value: _vehicleChargeable,
            ),
            RaisedButton(
              child: Text('Fahrzeug hinzufügen'),
              onPressed: () => onPressed,
              highlightColor: Theme.of(context).accentColor,
              color: Theme.of(context).primaryColor,
              colorBrightness: Theme.of(context).primaryColorBrightness,
            )
          ],
        ),
      ),
    );
  }

  String requiredValue(String str) {
    return str.isEmpty ? "Erforderlich" : null;
  }

  void onPressed() {
    var form = _formKey.currentState;

    if (form.validate()) {
      form.save();
    }
  }
}

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
