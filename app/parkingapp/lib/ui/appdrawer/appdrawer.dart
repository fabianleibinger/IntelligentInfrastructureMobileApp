import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/Provider.dart';

class AppDrawer extends StatelessWidget {
  AppDrawer(this.currentPage);

  // TODO Make this something dynamic
  final String currentPage;

  @override
  Widget build(BuildContext context) {
    var currentDrawer = Provider.of<DrawerStateInfo>(context).getCurrentDrawer;
    return Drawer(
      child: ListView(
        padding: EdgeInsets.zero,
        children: <Widget>[
          DrawerHeader(
            decoration: BoxDecoration(
              color: green,
            ),
            child: Text(AppLocalizations.of(context).drawerHeader,
                style: blackHeader),
          ),
          generateTile(context, currentDrawer, Routes.main, 'Main', Icons.message),
          generateTile(context, currentDrawer, Routes.vehicle, AppLocalizations.of(context).drawerVehicles, Icons.directions_car),
          generateTile(context, currentDrawer, Routes.settings, AppLocalizations.of(context).drawerSettings, Icons.settings),
        ],
      ),
    );
  }
}

ListTile generateTile(BuildContext context, String currentDrawer, String route, String text, IconData icon) {
  return ListTile(
    selected: currentDrawer == route ? true : false,
    leading: Icon(icon),
    title: Text(text),
    onTap: () {
      Navigator.pushReplacementNamed(context, route);
      Provider.of<DrawerStateInfo>(context, listen: false)
          .setCurrentDrawer(route);
    }
  );
}

class DrawerStateInfo with ChangeNotifier {
  String _currentDrawer = Routes.main;
  String get getCurrentDrawer => _currentDrawer;

  void setCurrentDrawer(String drawer) {
    _currentDrawer = drawer;
    notifyListeners();
  }

  void increment() {
    notifyListeners();
  }
}

//TODO implement
List<ListTile> generateVehicles(BuildContext context) {
  List<ListTile> tiles = [];
  tiles.add(
    ListTile(
        leading: Icon(Icons.message),
        title: Text('Messages'),
        onTap: () => Navigator.pushReplacementNamed(context, Routes.vehicle)),
  );
  return tiles;
}
