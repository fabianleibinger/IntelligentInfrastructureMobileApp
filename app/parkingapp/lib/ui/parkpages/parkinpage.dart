import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:parkingapp/util/parkmanager.dart';

class ParkInPage extends StatefulWidget {
  static const String routeName = '/parkinpage';

  final String carInAppKey;

  ParkInPage(this.carInAppKey);

  @override
  _ParkInPageState createState() => _ParkInPageState();
}

class _ParkInPageState extends State<ParkInPage> {
  Timer _timer;

  @override
  void initState() {
    //update parking spots
    currentParkingGarage.updateAllFreeParkingSpots();

    //init vehicle list
    DataHelper.initVehicles(context);
    BlocListener<VehicleBloc, List<Vehicle>>(
      listener: (context, vehicleList) {
        for (Vehicle vehicle in vehicleList) {
          print(vehicle.toString());
        }
      },
    );
    //get vehicle that shall be used from the list of vehicles
    for (Vehicle currentVehicle
        in BlocProvider.of<VehicleBloc>(context).state) {
      if (currentVehicle.inAppKey == widget.carInAppKey)
        vehicle = currentVehicle;
    }

    //add timer to update map
    _timer =
        Timer.periodic(Duration(milliseconds: 500), (timer) => setState(() {}));

    super.initState();
    //wait until build finished to call method
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkIn(context));
  }

  //TODO the timer will not be canceled (a propper check if parked in is needed)
  @override
  void dispose() {
    _timer.cancel();
    super.dispose();
  }

  //TODO: setState when parkedIn switches

  @override
  Widget build(BuildContext context) {
    //cancel the timer if the vehicle is parked in
    ParkManager.needsToParkIn(vehicle) ? null : _timer.cancel();

    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkIn),
      //button observes parkedIn value of car
      floatingActionButton: ValueListenableBuilder(
          valueListenable: vehicle.parkedInObserver,
          builder: (BuildContext context, bool, Widget child) {
            return FloatingActionButton.extended(
              //cancel or park out button
              label: vehicle.parkedIn
                  ? Text(AppLocalizations.of(context).actionButtonParkOut)
                  : Text(AppLocalizations.of(context).actionButtonCancelPark),
              backgroundColor: red,
              //park out dialog or cancel dialog
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
                ParkManager.getParkInAnimation(
                    context: context,
                    vehiclePosition: vehicle.location,
                    destination: vehicle.parkingSpot),
              ],
            ),
          )
        ],
      ),
    );
  }
}
