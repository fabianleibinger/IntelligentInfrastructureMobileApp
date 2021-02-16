import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/dialogs/chargetimedialog.dart';
import 'package:parkingapp/dialogs/chargingproviderdialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/dialogs/vehicledimensionsdialog.dart';
import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

Vehicle vehicle;
final currentParkingGarage = ParkingGarage(
    'Parkgarage Fasanengarten',
    ParkingGarageType.Tiefgarage,
    79,
    'assets/parkgarage-fasanengarten.jpg');
final parkingGarageImageHeight = 250;
final bottomMargin = 80;

class MainPage extends StatefulWidget {
  static const String routeName = '/MainPage';
  //final String apiKey;
  String carInAppKey;

  //const MainPage({Key key, this.apiKey}) : super(key: key);
  MainPage(String carInAppKey) {
    this.carInAppKey = carInAppKey;
    print('Vehicle page: carInAppKey: ' + this.carInAppKey);
  }

  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  void initState() {
    super.initState();
    DatabaseProvider.db.getVehicles().then((vehicles) {
      BlocProvider.of<VehicleBloc>(context).add(SetVehicles(vehicles));
    });
    BlocListener<VehicleBloc, List<Vehicle>>(
      listener: (context, vehicleList) {
        for (Vehicle vehicle in vehicleList) {
          print(vehicle.toString());
        }
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return BlocBuilder<VehicleBloc, List<Vehicle>>(
      buildWhen: (List<Vehicle> previous, List<Vehicle> current) {
        if (previous.length != current.length)
          return true;
        else
          return false;
      },
      builder: (context, vehicleList) {
        //get vehicle that shall be used from the list of vehicles
        for (Vehicle currentVehicle in vehicleList) {
          if (currentVehicle.inAppKey == widget.carInAppKey)
            vehicle = currentVehicle;
        }
        print('MainPage: vehicle: ' +
            vehicle.name +
            ' license plate: ' +
            vehicle.licensePlate);
        return Scaffold(
            appBar: AppBar(
              title: Text(vehicle.name, style: whiteHeader),
            ),
            drawer: AppDrawer(Routes.main),
            floatingActionButton: FloatingActionButton.extended(
              onPressed: () {
                showDialog(context: context, builder: (context) {
                  return ChargeTimeDialog();
                });
                //DatabaseProvider.db.clear();
              },
              label: Text(AppLocalizations.of(context).actionButtonPark),
            ),
            floatingActionButtonLocation:
                FloatingActionButtonLocation.centerFloat,
            body: Column(
              children: [
                Expanded(
                  child: Card(
                    margin: EdgeInsets.all(0),
                    elevation: 10,
                    clipBehavior: Clip.antiAlias,
                    child: Column(
                      children: [
                        ListTile(
                          //leading: Icon(Icons.location_on),
                          title: Text(currentParkingGarage.name),
                          subtitle:
                              Text(currentParkingGarage.type.toShortString()),
                        ),
                        Container(
                          height: parkingGarageImageHeight.toDouble(),
                          decoration: BoxDecoration(
                              image: DecorationImage(
                            image: AssetImage(currentParkingGarage.image),
                            fit: BoxFit.cover,
                          )),
                        ),
                        ListView(
                          shrinkWrap: true,
                          children: buildCarToggles(vehicle),
                        ),
                      ],
                    ),
                  ),
                ),
                Container(
                  height: MediaQuery.of(context).padding.bottom + bottomMargin,
                )
              ],
            ));
      },
    );
  }

  List<Widget> buildCarToggles(Vehicle vehicle) {
    // car park specific items
    List<String> _properties = [
      AppLocalizations.of(context).mainPageAvailableSpaces +
          currentParkingGarage.freeParkingSpots.toString()
    ];

    List<Widget> widgets = [];
    widgets.addAll(_properties
        .map((element) => ListTile(
              title: Text(element),
            ))
        .toList());

    // vehicle specific toggles
    widgets.add(SwitchListTile(
      title: Text(AppLocalizations.of(context).nearExitPreference),
      onChanged: (bool newValue) {
        setState(() => vehicle.nearExitPreference = newValue);
        DatabaseProvider.db.update(vehicle);
      },
      value: vehicle.nearExitPreference,
    ));

    widgets.add(SwitchListTile(
      title: Text(AppLocalizations.of(context).parkingCard),
      onChanged: (bool newValue) {
        setState(() => vehicle.parkingCard = newValue);
        DatabaseProvider.db.update(vehicle);
      },
      value: vehicle.parkingCard,
    ));

    // electric vehicle toggles
    if (vehicle.runtimeType == LoadableVehicle)
      widgets.addAll(addElectricVehicleTiles(vehicle));

    return widgets;
  }

  List<Widget> addElectricVehicleTiles(LoadableVehicle vehicle) {
    List<Widget> widgets = [];
    widgets.add(SwitchListTile(
      title:
          Text(AppLocalizations.of(context).mainPageCarPreferenceShouldCharge),
      onChanged: (bool newValue) {
        setState(() {
          vehicle.doCharge = newValue;
          DatabaseProvider.db.update(vehicle);
        });
      },
      value: vehicle.doCharge,
    ));
    return widgets;
  }
}
