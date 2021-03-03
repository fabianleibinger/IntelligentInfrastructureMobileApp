import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/resources/subtitleformatter.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkinggarageoccupieddialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

Vehicle vehicle;
ParkingGarage currentParkingGarage = ParkingGarage('Parkgarage Fasanengarten',
    ParkingGarageType.Tiefgarage, 79, 'assets/parkgarage-fasanengarten.jpg');

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
  final _parkingGarageImageHeight = 250;
  final _bottomMargin = 80;

  bool _buttonIsDisabled;

  @override
  void initState() {
    super.initState();
    _buttonIsDisabled = false;
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
  }

  //disables button according to free parking spots
  _setButtonIsDisabled() {
    bool disable = currentParkingGarage.getFreeParkingSpots() <= 0;
    setState(() {
      _buttonIsDisabled = disable;
    });
  }

  @override
  Widget build(BuildContext context) {
    //check if button should be disabled
    _setButtonIsDisabled();
    //get vehicle that shall be used from the list of vehicles
    for (Vehicle currentVehicle
        in BlocProvider.of<VehicleBloc>(context).state) {
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
        drawer: AppDrawer(),
        floatingActionButton: FloatingActionButton.extended(
          backgroundColor: _buttonIsDisabled
              ? Theme.of(context).disabledColor
              : Theme.of(context).primaryColor,
          onPressed: () {
            if (_buttonIsDisabled) {
              ParkingGarageOccupiedDialog.createDialog(context);
            } else {
              ParkDialog.createParkInDialog(context);
            }
          }
          //DatabaseProvider.db.clear();
          ,
          label: Text(AppLocalizations.of(context).actionButtonPark),
        ),
        floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
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
                      subtitle: Text(currentParkingGarage.type.toShortString()),
                    ),
                    Container(
                      height: _parkingGarageImageHeight.toDouble(),
                      decoration: BoxDecoration(
                          image: DecorationImage(
                        image: AssetImage(currentParkingGarage.image),
                        fit: BoxFit.cover,
                      )),
                    ),
                    Expanded(
                      child: ListView(
                        shrinkWrap: true,
                        children: buildCarToggles(vehicle),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            Container(
              height: MediaQuery.of(context).padding.bottom + _bottomMargin,
            )
          ],
        ));
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
    widgets.add(ListTile(
        title: Text(AppLocalizations.of(context).parkPreferences),
        subtitle: SubtitleFormatter.vehicleParkPreferences(
            context: context, vehicle: vehicle),
        onTap: () => _showDialog(context, ParkPreferencesDialog())));

    // electric vehicle toggles
    if (vehicle.runtimeType == ChargeableVehicle)
      widgets.addAll(addElectricVehicleTiles(vehicle));

    return widgets;
  }

  List<Widget> addElectricVehicleTiles(ChargeableVehicle vehicle) {
    List<Widget> widgets = [];
    widgets.add(SwitchListTile(
      title:
          Text(AppLocalizations.of(context).mainPageCarPreferenceShouldCharge),
      onChanged: (bool newValue) {
        setState(() {
          vehicle.setDoCharge(context, newValue);
        });
      },
      value: vehicle.doCharge,
    ));
    return widgets;
  }

  void _showDialog(BuildContext context, Widget dialog) async {
    await showDialog(context: context, builder: (context) => dialog);
    setState(() {});
  }
}
