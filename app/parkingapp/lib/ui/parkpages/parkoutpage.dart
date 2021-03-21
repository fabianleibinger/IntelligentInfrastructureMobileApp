import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkOutPage extends StatefulWidget {
  static const String routeName = '/parkoutpage';

  /// The inAppKey of the currently selected vehicle on [MainPage].
  final String carInAppKey;

  const ParkOutPage(this.carInAppKey);

  @override
  _ParkOutPageState createState() => _ParkOutPageState();
}

class _ParkOutPageState extends State<ParkOutPage> {

  /// Initializes selected vehicle and calls [vehicle.parkOut(context)].
  @override
  void initState() {
    super.initState();

    // Init vehicle list.
    DataHelper.initVehicles(context);
    BlocListener<VehicleBloc, List<Vehicle>>(
      listener: (context, vehicleList) {
        for (Vehicle vehicle in vehicleList) {
          print(vehicle.toString());
        }
      },
    );
    // Get vehicle that shall be used from the list of vehicles.
    for (Vehicle currentVehicle
    in BlocProvider.of<VehicleBloc>(context).state) {
      if (currentVehicle.inAppKey == widget.carInAppKey)
        vehicle = currentVehicle;
    }

    // Wait until build finished to call method.
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkOut(context));
  }

  /// Returns [Scaffold], Animation, [FloatingActionButton].
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkOut),
      // Button observes parkedIn value of car.
      floatingActionButton: ValueListenableBuilder(
        valueListenable: vehicle.parkedInObserver,
        builder: (BuildContext context, bool, Widget child) {
          return FloatingActionButton.extended(
            // Cancel or park out process button, not clickable.
            label: vehicle.parkedIn
                ? Text(AppLocalizations.of(context).actionButtonParkOutProcess)
                : Text(
                    AppLocalizations.of(context).actionButtonCancelParkProcess),
            backgroundColor: grey,
            onPressed: () {},
          );
        },
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: ListTile(
        title: Text(currentParkingGarage.name),
        subtitle: Text(currentParkingGarage.type.toShortString()),
      ),
    );
  }
}
