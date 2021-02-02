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
          ListTile(
            selected: currentDrawer == Routes.main ? true : false,
            leading: Icon(Icons.message),
            title: Text('Messages'),
            onTap: () {
              Navigator.pushReplacementNamed(context, Routes.main);
              Provider.of<DrawerStateInfo>(context, listen: false)
                  .setCurrentDrawer(Routes.main);
            },
          ),
          ListTile(
            selected: currentDrawer == Routes.vehicle ? true : false,
            leading: Icon(Icons.directions_car),
            title: Text(AppLocalizations.of(context).drawerVehicles),
            onTap: () {
              Navigator.pushReplacementNamed(context, Routes.vehicle);
              Provider.of<DrawerStateInfo>(context, listen: false)
                  .setCurrentDrawer(Routes.vehicle);
            },
          ),
          ListTile(
            selected: currentDrawer == Routes.settings ? true : false,
            leading: Icon(Icons.settings),
            title: Text(AppLocalizations.of(context).drawerSettings),
            onTap: () {
              Navigator.pushReplacementNamed(context, Routes.settings);
              Provider.of<DrawerStateInfo>(context, listen: false)
                  .setCurrentDrawer(Routes.settings);
            },
          ),
        ],
      ),
    );
  }
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
