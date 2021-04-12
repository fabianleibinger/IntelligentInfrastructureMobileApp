import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/Provider.dart';
import 'package:flutter_bloc/flutter_bloc.dart';

final EdgeInsets drawerHeaderPadding = EdgeInsets.all(16.0);
final EdgeInsets listViewPadding = EdgeInsets.fromLTRB(0, 8, 0, 0);

/// Unified AppDrawer used throughout the app
/// Displays the Apps Name in the header and the [Vehicle]s, [VehiclePage] and [SettingsPage] in a [ListView]
class AppDrawer extends StatelessWidget {
  AppDrawer([this.currentPage]);

  final String currentPage;

  @override
  Widget build(BuildContext context) {
    var currentDrawer = Provider.of<DrawerStateInfo>(context).getCurrentDrawer;
    return Drawer(
      child: Column(
        children: [
          //Drawer Header
          Container(
            height: MediaQuery.of(context).padding.top,
            color: Theme.of(context).primaryColor,
          ),
          Container(
            padding: drawerHeaderPadding,
            width: MediaQuery.of(context).size.width,
            child:
                Text(AppLocalizations.of(context).appName, style: whiteHeader),
            decoration: BoxDecoration(
              color: Theme.of(context).primaryColor,
              border: Border(
                bottom: Divider.createBorderSide(context),
              ),
            ),
          ),
          //Drawer Content
          Expanded(
            child: ListView(
              padding: listViewPadding,
              children: <Widget>[
                generateVehicles(context, currentDrawer),
                Divider(),
                //Edit vehicles
                generateTile(context, currentDrawer, Routes.vehicle,
                    AppLocalizations.of(context).drawerVehicles, Icons.edit),
                //Settings
                generateTile(
                    context,
                    currentDrawer,
                    Routes.settings,
                    AppLocalizations.of(context).drawerSettings,
                    Icons.settings),
              ],
            ),
          )
        ],
      ),
    );
  }
}

/// Generates a single standard tile for the [ListView] in the [AppDrawer]
/// [route] to page that shall be opened on tap
/// [text] shown on tile
/// [icon] shown to the left of the text on the tile
ListTile generateTile(BuildContext context, String currentDrawer, String route,
    String text, IconData icon) {
  return ListTile(
      selected: currentDrawer == route ? true : false,
      leading: Icon(icon),
      title: Text(text),
      onTap: () {
        currentDrawer == route
            ? Navigator.pop(context)
            : Navigator.pushReplacementNamed(context, route);
        //This may be added to the Class itself to update the [DrawerStateInfo] every time the respective widget is built
        Provider.of<DrawerStateInfo>(context, listen: false)
            .setCurrentDrawer(route);
      });
}

/// Generates a single vehicle tile for the [ListView] in the [AppDrawer] displaying [vehicle.name] and arouting to the vhicle page by [vehicle.inAppKey]
/// [icon] shown to the left of the text on the tile
ListTile generateVehicleTile(BuildContext context, String currentDrawer,
    Vehicle vehicle, IconData icon) {
  return ListTile(
    selected: currentDrawer == vehicle.inAppKey ? true : false,
    leading: Icon(icon),
    title: Text(vehicle.name),
    onTap: () {
      currentDrawer == vehicle.inAppKey
          ? Navigator.pop(context)
          : Navigator.pushReplacementNamed(
              context, Routes.returnCorrectRouteForVehicle(vehicle));
      //This may be added to the Class itself to update the [DrawerStateInfo] every time the respective widget is built
      Provider.of<DrawerStateInfo>(context, listen: false)
          .setCurrentDrawer(vehicle.inAppKey);
    },
  );
}

/// Generates [generateVehicleTile] for all [Vehicle] in the [VehicleBloc]
Widget generateVehicles(BuildContext context, String currentDrawer) {
  return BlocBuilder<VehicleBloc, List<Vehicle>>(
    buildWhen: (List<Vehicle> previous, List<Vehicle> current) {
      return true;
    },
    builder: (context, vehicleList) {
      //build vehicles Column
      List<ListTile> listTiles = [];
      for (Vehicle vehicle in vehicleList) {
        var icon;
        vehicle.runtimeType == ChargeableVehicle
            ? icon = Icons.electric_car
            : icon = Icons.directions_car;
        listTiles
            .add(generateVehicleTile(context, currentDrawer, vehicle, icon));
      }
      return Column(
        children: listTiles,
      );
    },
  );
}

/// this manages the highlighted item in the drawer.
class DrawerStateInfo with ChangeNotifier {
  // set initial drawer state
  //if not provided drawer will only highlight after the first interaction with the drawer
  DrawerStateInfo([this._currentDrawer]);

  String _currentDrawer;

  String get getCurrentDrawer => _currentDrawer;

  void setCurrentDrawer(String drawer) {
    _currentDrawer = drawer;
    notifyListeners();
  }

  void increment() {
    notifyListeners();
  }
}
