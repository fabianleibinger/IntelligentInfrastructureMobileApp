import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/bloc/resources/subtitleformatter.dart';
import 'package:parkingapp/dialogs/noconnectiondialog.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/dialogs/parkinggarageoccupieddialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

Vehicle vehicle;
ParkingGarage currentParkingGarage;

/// The main page allowing for parking in a [Vehicle]
// ignore: must_be_immutable
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
  final int _parkingGarageImageHeight = 250;
  final int _bottomMargin = 80;

  /// The variables that define the current state of the page.
  int _parkingSpots;
  bool _buttonIsDisabled;
  bool _noConnection;

  /// Sets initial state variables values and selects current vehicle.
  @override
  void initState() {
    super.initState();
    setState(() {
      currentParkingGarage.updateAllFreeParkingSpots();
      _parkingSpots = currentParkingGarage.freeParkingSpots;
      _noConnection = true;
      ApiProvider.connect().then((value) {
        _noConnection = false;
        _setButtonIsDisabled();
      }).catchError((e) => _setButtonIsDisabled());
    });
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

  // Only call setState() if widget is mounted.
  @override
  void setState(fn) {
    if (mounted) {
      super.setState(fn);
    }
  }

  /// Disables button.
  _setButtonIsDisabled() {
    setState(() {
      _buttonIsDisabled = disableButton();
    });
  }

  /// Check if button needs to be disabled.
  bool disableButton() {
    return _parkingSpots <= 0 || _noConnection;
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
    //check which parking spots should be displayed and if button should be disabled
    _parkingSpots = currentParkingGarage.getFreeSpotsForVehicle(vehicle);
    _buttonIsDisabled = disableButton();
    return Scaffold(
        appBar: AppBar(
          title: Text(vehicle.name),
        ),
        drawer: AppDrawer(),
        floatingActionButton: FloatingActionButton.extended(
          backgroundColor: _buttonIsDisabled
              ? Theme.of(context).disabledColor
              : Theme.of(context).primaryColor,
          onPressed: () {
            if (_buttonIsDisabled) {
              if (_noConnection) {
                showDialog(
                    context: context,
                    builder: (context) {
                      return NoConnectionDialog.getDialog(context);
                    });
              } else {
                showDialog(
                    context: context,
                    builder: (context) {
                      return ParkingGarageOccupiedDialog.getDialog(context);
                    });
              }
            } else {
              showDialog(
                  context: context,
                  builder: (context) {
                    return ParkDialogs.getParkInDialog(context);
                  });
            }
          },
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

  /// Build all [ListTile]s for a [Vehicle] that can be changed on the [MainPage]
  List<Widget> buildCarToggles(Vehicle vehicle) {
    // car park specific items
    List<String> _properties;
    _noConnection
        ? _properties = [AppLocalizations.of(context).noConnectionDialogTitle]
        : _properties = [
            AppLocalizations.of(context).mainPageAvailableSpaces +
                _parkingSpots.toString()
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

  /// Build all [ListTile]s for a [ChargeableVehicle] that can be changed on the [MainPage]
  List<Widget> addElectricVehicleTiles(ChargeableVehicle vehicle) {
    List<Widget> widgets = [];
    widgets.add(SwitchListTile(
      title:
          Text(AppLocalizations.of(context).mainPageCarPreferenceShouldCharge),
      onChanged: (bool newValue) {
        setState(() {
          vehicle.setAndUpdateDoCharge(context, newValue);
          if (newValue) {
            _parkingSpots = currentParkingGarage.freeChargeableParkingSpots;
          } else {
            _parkingSpots = currentParkingGarage.freeParkingSpots;
          }
          _setButtonIsDisabled();
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
