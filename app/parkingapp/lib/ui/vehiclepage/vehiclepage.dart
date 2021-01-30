import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/vehicleevent.dart';
import 'package:parkingapp/dialogs/scanqrdialog.dart';
import 'package:parkingapp/dialogs/drivesourcedialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/util/utility.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
// example for a page (mainpage)

class VehiclePage extends StatefulWidget {
  static const String routeName = '/vehiclepage';
  const VehiclePage({Key key}) : super(key: key);
  @override
  _VehiclePageState createState() => _VehiclePageState();
}

class _VehiclePageState extends State<VehiclePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Vehicle', style: whiteHeader),
      ),
      drawer: AppDrawer(),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () {
          BlocProvider.of<VehicleBloc>(context).add(VehicleEvent.add(Vehicle(
              Utility.generateKey(),
              "Audi",
              "OG-DE-923",
              93.0,
              93.4,
              29.3,
              84.0,
              true,
              false)));
        },
        label: Text(AppLocalizations.of(context).actionButtonPark),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: Text("Hello"),
    );
  }
}
