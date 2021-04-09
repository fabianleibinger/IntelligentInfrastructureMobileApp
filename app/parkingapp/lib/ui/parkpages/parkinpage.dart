import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:parkingapp/util/parkmanager.dart';

class ParkInPage extends StatefulWidget {
  static const String routeName = '/parkinpage';

  /// The inAppKey of the currently selected vehicle on [MainPage].
  final String carInAppKey;

  ParkInPage(this.carInAppKey);

  @override
  _ParkInPageState createState() => _ParkInPageState();
}

class _ParkInPageState extends State<ParkInPage> {
  /// Initializes selected vehicle and calls [vehicle.parkIn(context)].
  @override
  void initState() {
    // Update parking spots.
    currentParkingGarage.updateAllFreeParkingSpots();

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
    super.initState();
    // Wait until build finished to call method.
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkIn(context));
  }

  /// Returns [Scaffold], Animation, park out button [FloatingActionButton].
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name)),
      drawer: AppDrawer(Routes.parkIn),
      // Button observes parkedIn value of car.
      floatingActionButton: ValueListenableBuilder(
          valueListenable: vehicle.parkedInObserver,
          builder: (BuildContext context, bool, Widget child) {
            return FloatingActionButton.extended(
              // Cancel or park out button.
              label: vehicle.parkedIn
                  ? Text(AppLocalizations.of(context).actionButtonParkOut)
                  : Text(AppLocalizations.of(context).actionButtonCancelPark),
              backgroundColor: red,
              // Park out dialog or cancel dialog
              onPressed: () {
                showDialog(
                    context: context,
                    builder: (context) {
                      if (vehicle.parkedIn) {
                        return ParkDialogs.getParkOutDialog(context);
                      } else {
                        return ParkDialogs.getParkInCancelDialog(context);
                      }
                    });
              },
            );
          }),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: Column(
        children: [
          ListTile(
            title: Text(currentParkingGarage.name),
            subtitle: Text(currentParkingGarage.type.toShortString()),
          ),
          //add the map
          Expanded(
            child: ListView(
              children: [
                ValueListenableBuilder(
                    valueListenable: vehicle.locationObserver,
                    builder: (BuildContext context, coordinate, Widget widget) {
                      print('rebuilding park in map');
                      return ParkManager.getParkAnimation(
                          context: context,
                          vehiclePosition: vehicle.location,
                          destination: vehicle.parkingSpot);
                    })
              ],
            ),
          )
        ],
      ),
    );
  }
}
