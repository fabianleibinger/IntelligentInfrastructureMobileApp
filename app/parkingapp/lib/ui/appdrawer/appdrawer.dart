import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/Provider.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';

final EdgeInsets drawerHeaderPadding = EdgeInsets.all(16.0);
final EdgeInsets listViewPadding = EdgeInsets.fromLTRB(0, 8, 0, 0);

class AppDrawer extends StatelessWidget {
  AppDrawer(this.currentPage);

  // TODO Make this something dynamic
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
            child: Text(AppLocalizations.of(context).drawerHeader,
                style: whiteHeader),
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

//route: Route to page that shall be opened on tap
//text: text shown on tile
//icon: icon shown to the left of the text on the tile
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

Widget generateVehicles(BuildContext context, String currentDrawer) {
  // get vehicleList
  DatabaseProvider.db.getVehicles().then((vehicleList) {
    BlocProvider.of<VehicleBloc>(context).add(SetVehicles(vehicleList));
  });
  return BlocBuilder<VehicleBloc, List<Vehicle>>(
    buildWhen: (List<Vehicle> previous, List<Vehicle> current) {
      return true;
    },
    builder: (context, vehicleList) {
      //build vehicles Column
      //TODO Sort list
      List<ListTile> listTiles = [];
      //this sometimes seems to fail with
      /*The following NoSuchMethodError was thrown building BlocBuilder<VehicleBloc, List<Vehicle>>(dirty, dependencies: [MediaQuery, _LocalizationsScope-[GlobalKey#55074]], state: _BlocBuilderBaseState<VehicleBloc, List<Vehicle>>#f9ab8):
      The getter 'name' was called on null.
      Receiver: null
      Tried calling: name

      The relevant error-causing widget was
      BlocBuilder<VehicleBloc, List<Vehicle>>
      package:parkingapp/…/mainpage/mainpage.dart:62
      When the exception was thrown, this was the stack
      #0      Object.noSuchMethod (dart:core-patch/object_patch.dart:51:5)
      #1      _MainPageState.build.<anonymous closure>
      package:parkingapp/…/mainpage/mainpage.dart:77
      #2      BlocBuilder.build
      package:flutter_bloc/src/bloc_builder.dart:93
      #3      _BlocBuilderBaseState.build
      package:flutter_bloc/src/bloc_builder.dart:153
      #4      StatefulElement.build
      */
      for (Vehicle vehicle in vehicleList) {
        listTiles.add(generateTile(context, currentDrawer, vehicle.inAppKey,
            vehicle.name, Icons.directions_car));
      }
      return Column(
        children: listTiles,
      );
    },
  );
}

// this manages the highlighted item in the drawer.
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
