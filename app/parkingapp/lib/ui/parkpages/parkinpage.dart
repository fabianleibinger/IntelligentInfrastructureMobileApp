import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkInPage extends StatefulWidget {
  static const String routeName = '/parkinpage';
  Coordinate vehiclePosition;

  /// The inAppKey of the currently selected vehicle on [MainPage].
  final String carInAppKey;

  ParkInPage(this.carInAppKey);

  @override
  _ParkInPageState createState() => _ParkInPageState();
}

class _ParkInPageState extends State<ParkInPage> {
  //variables needed for the overlay
  final controller = ScrollController();
  OverlayEntry sticky;
  GlobalKey stickyKey = GlobalKey();

  /// Initializes selected vehicle and calls [vehicle.parkIn(context)].
  @override
  void initState() {
    // Update parking spots.
    currentParkingGarage.updateAllFreeParkingSpots();

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
    new Timer.periodic(Duration(seconds: 5), (timer) => setState(() {}));

    super.initState();
    // Wait until build finished to call method.
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkIn(context));
  }

  //TODO: setState when parkedIn switches

  /// Returns [Scaffold], Animation, park out button [FloatingActionButton].
  @override
  Widget build(BuildContext context) {
    ApiProvider.getPosition(vehicle).then((value) {
      double latitude = value["latitude"];
      double longitude = value["longitude"];
      widget.vehiclePosition =
          Coordinate(latitude: latitude, longitude: longitude);
      print(widget.vehiclePosition);
    });

    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
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
                getParkInAnimation(
                    context: context, vehiclePosition: widget.vehiclePosition),
              ],
            ),
          )
        ],
      ),
    );
  }

  //the vehicle icon overlay
  getParkInAnimation({BuildContext context, Coordinate vehiclePosition}) {
    //TODO height must be calculated from aspect ratio of mapp
    final double _width = MediaQuery.of(context).size.width;
    final double _height = (1473 * _width) / 1000;
    print(_width.toString() + ' x ' + _height.toString());

    //assume 0x0 to be the bottom left
    Coordinate _topRightAdjusted = Coordinate(
        longitude: currentParkingGarage.topRight.longitude -
            currentParkingGarage.bottomLeft.longitude,
        latitude: currentParkingGarage.topRight.latitude -
            currentParkingGarage.bottomLeft.latitude);
    print(_topRightAdjusted);

    double _iconSize = 16;
    Container _icon = Container(
        width: _iconSize, height: _iconSize, child: Icon(Icons.circle));

    //set adjusted vehicle position if not null
    Coordinate _vehiclePositionAdjusted;
    double iconOffsetHeight, iconOffsetWidth;
    Positioned _positionedIcon;
    if (vehiclePosition != null) {
      _vehiclePositionAdjusted = Coordinate(
          latitude: vehiclePosition.latitude -
              currentParkingGarage.bottomLeft.latitude,
          longitude: vehiclePosition.longitude -
              currentParkingGarage.bottomLeft.longitude);

      //scale the icons position
      iconOffsetHeight = (_height / _topRightAdjusted.latitude) *
              _vehiclePositionAdjusted.latitude -
          (_iconSize / 2);
      iconOffsetWidth = (_width / _topRightAdjusted.longitude) *
              _vehiclePositionAdjusted.longitude -
          (_iconSize / 2);

      //icon
      _positionedIcon = Positioned(
        left: iconOffsetWidth,
        bottom: iconOffsetHeight,
        child: _icon,
      );
    }

    //empty positioned
    if (_positionedIcon == null)
      _positionedIcon = Positioned(
        child: Container(),
      );

    return Stack(alignment: Alignment.center, children: [
      //map
      Container(
        width: _width,
        height: _height,
        child: Image(
          image: AssetImage(currentParkingGarage.map),
          fit: BoxFit.fill,
        ),
      ),
      //icon
      _positionedIcon
    ]);
  }
}
