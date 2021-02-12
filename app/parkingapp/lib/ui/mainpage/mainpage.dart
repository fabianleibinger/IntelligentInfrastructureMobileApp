import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkoutdialog.dart';
import 'package:parkingapp/enum/parkinggaragetype.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/ui/parkpage/parkpage.dart';
import 'package:parkingapp/util/utility.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

final currentParkingGarage = ParkingGarage('Parkgarage Fasanengarten',
    ParkingGarageType.Tiefgarage, 79, 'assets/parkgarage-fasanengarten.jpg');
final parkingGarageImageHeight = 250;
final bottomMargin = 220;
bool _charge = false;

class MainPage extends StatefulWidget {
  static const String routeName = '/MainPage';
  final String apiKey;

  const MainPage({Key key, this.apiKey}) : super(key: key);

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
    // TODO generate from vehicle
    List<String> _properties = [
      AppLocalizations.of(context).mainPageAvailableSpaces +
          currentParkingGarage.freeParkingSpots.toString(),
      AppLocalizations.of(context).mainPageCarPreferences +
          AppLocalizations.of(context).textNone
    ];
    return Scaffold(
        appBar: AppBar(
          title: Text('Vehicle', style: whiteHeader),
        ),
        drawer: AppDrawer(),
        floatingActionButton: FloatingActionButton.extended(
          onPressed: () {
            DatabaseProvider.db.clear();
          },
          label: Text(AppLocalizations.of(context).actionButtonPark),
        ),
        floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
        body: Column(
          children: [
            Container(
              constraints: BoxConstraints(
                  maxHeight: MediaQuery.of(context).size.height - bottomMargin),
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
                      height: parkingGarageImageHeight.toDouble(),
                      decoration: BoxDecoration(
                          image: DecorationImage(
                        image: AssetImage(currentParkingGarage.image),
                        fit: BoxFit.cover,
                      )),
                    ),
                    Flexible(
                      child: ListView(
                        shrinkWrap: true,
                        children: buildElements(_properties),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ));
  }

  List<Widget> buildElements(List<String> elements) {
    List<Widget> widgets = [];
    widgets.addAll(elements
        .map((element) => ListTile(
              title: Text(element),
            ))
        .toList());
    widgets.add(SwitchListTile(
      title:
          Text(AppLocalizations.of(context).mainPageCarPreferenceShouldCharge),
      onChanged: (bool newValue) {
        setState(() {
          _charge = newValue;
        });
      },
      value: _charge,
    ));
    return widgets;
  }
}
