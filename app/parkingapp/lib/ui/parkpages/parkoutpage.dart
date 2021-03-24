import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:parkingapp/util/parkmanager.dart';

class ParkOutPage extends StatefulWidget {
  static const String routeName = '/parkoutpage';
  Coordinate vehiclePosition;

  final String carInAppKey;

  ParkOutPage(this.carInAppKey);

  @override
  _ParkOutPageState createState() => _ParkOutPageState();
}

class _ParkOutPageState extends State<ParkOutPage> {
  Timer _timer;

  @override
  void initState() {
    super.initState();

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
    _timer = Timer.periodic(Duration(seconds: 5), (timer) => setState(() {}));

    //wait until build finished to call method
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkOut(context));
  }

  @override
  void dispose() {
    _timer.cancel();
    super.dispose();
  }

  //TODO: switch _firstBuild to false in setState()

  @override
  Widget build(BuildContext context) {
    //update position
    ParkManager.getVehiclePosition(vehicle)
        .then((coordinate) => widget.vehiclePosition = coordinate);
    //cancel the timer if the vehicle is parked in
    ParkManager.needsToParkOut(vehicle) ? null : _timer.cancel();

    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkOut),
      //button observes parkedIn value of car
      floatingActionButton: ValueListenableBuilder(
        valueListenable: vehicle.parkedInObserver,
        builder: (BuildContext context, bool, Widget child) {
          return FloatingActionButton.extended(
            //cancel or park out process button, not clickable
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
                    context: context, vehiclePosition: widget.vehiclePosition),
              ],
            ),
          )
        ],
      ),
    );
  }
}
